<p align="right">
  <a href="README.md">English</a> | <a href="README.zh-CN.md">中文</a>
</p>

# stdpp::event

A lightweight, type-safe, modern C++ event toolkit for callback aggregation, key-based dispatching, and queued batch processing with asynchronous waiting support.

It provides **five composable primitives** ranging from zero-overhead `FastEvent` to fully-thread-safe `QueueDispatcher`, all connected through a uniform `Handle` API.

## Table of Contents

- [Components](#components)
- [FastEvent — Zero-Overhead Lightweight](#fastevent--zero-overhead-lightweight)
- [Event — Aggregated Callbacks](#event--aggregated-callbacks)
- [Dispatcher — Key-Based Dispatching](#dispatcher--key-based-dispatching)
- [EventQueue — Buffered Batch Processing](#eventqueue--buffered-batch-processing)
- [QueueDispatcher — Queued + Key-Based Dispatching](#queuedispatcher--queued--key-based-dispatching)
- [Handle Semantics](#handle-semantics)
- [Subscription Removal](#subscription-removal)
- [Thread Safety](#thread-safety)
- [Design Notes](#design-notes)

---

## Components

| Component                          | Dispatch  | Queue      | Return Value              | Thread Safety | Overhead                   |
| ---------------------------------- | --------- | ---------- | ------------------------- | ------------- | -------------------------- |
| `FastEvent<R(Args...)>`            | broadcast | —          | ❌                        | ❌            | zero-overhead (raw fn ptr) |
| `Event<R(Args...)>`                | broadcast | —          | `Handle::last()`          | ✅            | moderate                   |
| `Dispatcher<Key, R(Args...)>`      | by key    | —          | `Handle::last()`          | ✅            | moderate                   |
| `EventQueue<R(Args...)>`           | broadcast | ✅ enqueue | `Handle::last()` (vector) | ✅            | moderate                   |
| `QueueDispatcher<Key, R(Args...)>` | by key    | ✅ enqueue | `Handle::last()` (vector) | ✅            | moderate                   |

---

## FastEvent — Zero-Overhead Lightweight

> High-frequency, lock-free event invocation using **raw function pointers** with **generation-based slot reuse**.

```cpp
stdpp::event::FastEvent<void(int)> fast;

auto h1 = fast.append([](int v) { /* ... */ });
auto h2 = fast += [](int v) { /* ... */ };

fast(42);          // invoke all callbacks
fast.remove(h1);   // remove by Handle
fast -= h2;        // remove by Handle
```

- Uses **raw function pointers** (`R(*)(Args...)`) — no `std::function` overhead
- **Generation-based slot management** prevents ABA problems
- `free_list` recycles removed slots
- **Not thread-safe** — designed for single-threaded hot paths
- Returns `void` only (no return value collection)

---

## Event — Aggregated Callbacks

> Collect and invoke multiple callbacks, each with an individual return value.

```cpp
stdpp::event::Event<int(int, int)> process;

auto h1 = process.append([](int a, int b) { return a + b; });
auto h2 = process += [](int a, int b) { return a + b + a; };

// Async invocation with result sync
std::thread([&] {
    std::this_thread::sleep_for(5s);
    process(1, 2);                // invoke all callbacks sequentially
}).detach();

h1.wait();                        // wait for first execution
auto r1 = h1.last();              // std::optional<int>
```

- `operator()` invokes all callbacks **sequentially**, returns the number invoked
- Each callback's result is cached independently in its `Handle::last()`
- `Handle::wait()` blocks until the next execution completes
- Thread-safe: callback list is **snapshot-copied** under shared_lock before invocation

---

## Dispatcher — Key-Based Dispatching

> Route events to different subscribers by key.

```cpp
stdpp::event::Dispatcher<int, void(int)> disp;

auto h = disp.subscribe(42, [](int x) {
    // handle event for key 42
});

disp(42, 7);               // trigger only subscribers of key 42

disp.remove(42);           // remove all subscribers of key 42
disp.remove(h);            // remove by Handle (precise)
```

- A single key may have **multiple subscribers**
- Supports **global subscribers** that receive events for all keys
- `operator()` returns the number of callbacks invoked
- `Handle::last()` and `Handle::wait()` work the same as `Event`

---

## EventQueue — Buffered Batch Processing

> Enqueue events without immediate execution; process them in batches.

```cpp
stdpp::event::EventQueue<std::string(std::string, bool)> queue;

auto h = queue.append([](std::string s, bool ok) {
    return ok ? std::move(s) : "";
});

queue.enqueue("Hello", true);     // buffer only, no execution
queue.enqueue("Hello", false);

std::thread([&] {
    std::this_thread::sleep_for(3s);
    queue();                       // process all queued items
}).detach();

h.wait();
auto results = h.last();           // std::vector<std::string>
```

- `enqueue()` buffers arguments — **does not trigger anything**
- `operator()` processes the queue in batch
- `Handle::last()` drains all accumulated results at once
- `MaxResults` template parameter controls the result queue capacity (default 1024)

---

## QueueDispatcher — Queued + Key-Based Dispatching

> Combine key-based routing with queued batch processing.

```cpp
stdpp::event::QueueDispatcher<int, int(int)> qd;

auto h = qd.subscribe(1, [](int v) { return v * 2; });

qd.enqueue(1, 10);                 // buffer under key 1
qd.enqueue(1, 20);
qd.enqueue(1, 30);

std::thread([&] {
    std::this_thread::sleep_for(3s);
    qd(1);                         // process only queue for key 1
}).detach();

h.wait();
auto results = h.last();           // {20, 40, 60}
```

- Combines `Dispatcher` routing + `EventQueue` buffering
- Each key has its own event queue
- Supports global subscriptions (no key)

---

## Handle Semantics

Every call to `append()` / `subscribe()` returns a **Handle**:

```cpp
auto h = event.append(callback);
```

| API             | Description                               |
| --------------- | ----------------------------------------- |
| `wait()`        | Block until the next execution completes  |
| `wait(timeout)` | Block with timeout                        |
| `last()`        | Retrieve the most recent execution result |
| Auto-unbind     | Automatically unsubscribed on destruction |

**Important `wait()` semantics by component:**

| Component                        | `wait()` means                                |
| -------------------------------- | --------------------------------------------- |
| `Event` / `Dispatcher`           | Waits for **one full invocation** to complete |
| `EventQueue` / `QueueDispatcher` | Waits for **one queued item** to complete     |

> For queue-based components, `wait()` unblocks as soon as **any one** queued item finishes. Use a loop to drain the entire batch:
>
> ```cpp
> for (int i = 0; i < 3; ++i) {
>     h.wait();
>     auto r = h.last();
> }
> ```

**Handle internally holds a `weak_ptr`** — it never extends the lifetime of the callback node. If the event is destroyed or the callback removed, all Handle operations become safe no-ops.

---

## Subscription Removal

All components support removal by **function pointer** or by **Handle**:

```cpp
// By function pointer (removes all matching callbacks)
event.remove(&my_func);
event -= &my_func;

// By Handle (precise, recommended)
event.remove(h);
event -= h;
```

### Dispatcher-specific removal

```cpp
disp.remove(42, &my_func);   // remove under specific key
disp.remove(&my_func);       // remove from all keys
disp.remove(42);              // remove entire key
disp.remove(h);               // by Handle
```

### Queue-based removal

```cpp
queue.remove(&process);
queue.remove(h);

qd.remove(1, &process);
qd.remove(&process);
qd.remove(1);
qd.remove(h);
```

- Removal only affects **future executions**
- Items already enqueued but not yet processed are skipped if no callbacks remain

---

## Thread Safety

| Operation             | Lock                                            | Details                  |
| --------------------- | ----------------------------------------------- | ------------------------ |
| Add / remove callback | `unique_lock(mutex)`                            | Exclusive write          |
| Invoke (`operator()`) | `shared_lock(mutex)` + snapshot copy            | Concurrent reads allowed |
| Read `last()`         | `shared_lock(node->mutex)`                      | Non-blocking             |
| Write `last()`        | `unique_lock(node->mutex)`                      | During invocation        |
| `wait()`              | `unique_lock(node->mutex)` + condition variable |                          |
| `enqueue()`           | `unique_lock(queue_mutex)`                      | Queue only               |

> **Exception safety:** `operator()` wraps each callback invocation in try-catch. If a callback throws, `last_value` is reset and execution continues with the next callback.

---

## Design Notes

- Header-only C++17, no external dependencies beyond the standard library
- `Handle` uses `weak_ptr` internally — safe against dangling references
- `FastEvent` uses raw function pointers + generation-based slots for zero-overhead hot paths
- Queue-based components have a `MaxResults` template parameter (default 1024); oldest results are evicted when the limit is reached
- Callback list is snapshot-copied on invocation — adding/removing during dispatch is safe and takes effect next time
