# stdpp::event

> A lightweight, type-safe, modern C++ event / callback / queue dispatching toolkit with asynchronous waiting support.

`stdpp::event` provides a set of **composable event primitives** designed to solve the following problems:

* Unified management and invocation of multiple callbacks
* Collection and synchronization of callback execution results
* Key-based event dispatching (`Dispatcher`)
* Queued batched event processing (`EventQueue`)
* Combined queue + dispatch model (`QueueDispatcher`)
* High-frequency, lock-free event invocation path (`FastEvent`)

All components support:

* **Thread safety**
* **Asynchronous execution with synchronous waiting**
* **Cached results (`last`)**

---

## Features Overview

* `Event<Signature>`: Aggregated invocation of multiple callbacks
* `Dispatcher<Key, Signature>`: Key-based event dispatching
* `EventQueue<Signature>`: Parameterized queued batch processing
* `QueueDispatcher<Key, Signature>`: Queued dispatching by key
* Each subscription / callback owns a **Handle**
* `wait()` / `last()` for synchronization and result retrieval

---

## Event — Aggregated Callbacks

### Event with Return Value

```cpp
stdpp::event::Event<int(int, int)> process;

auto h1 = process.append([](int a, int b) {
    return a + b;
});

auto h2 = process += ([](int a, int b) {
    return a + b + a;
});

// auto h2 = process + ([](int a, int b) {
//     return a + b + a;
// });
```

Asynchronous invocation with result synchronization:

```cpp
std::thread([&] {
    std::this_thread::sleep_for(5s);
    auto invoked = process(1, 2);   // invoke all callbacks
}).detach();

h1.wait();                      // wait for execution

auto r1 = h1.last();            // std::optional<int>
auto r2 = h2.last();
```

* `process(1, 2)` invokes **all callbacks sequentially**
* Each return value is cached in its corresponding `Handle`
* `wait()` blocks until the first execution completes

---

### Sequential Callback Invocation

`Event<std::string(int, int)>`

```cpp
stdpp::event::Event<std::string(int, int)> calls;

calls.append([](int a, int b) -> std::string {
    return std::to_string(a + b);
});
```

```cpp
std::thread([&] {
    std::this_thread::sleep_for(3s);
    calls();
}).detach();

h.wait();
std::optional<std::string> str = h.last();
```

---

## Dispatcher — Key-Based Event Dispatching

```cpp
stdpp::event::Dispatcher<int, void(int)> disp;

auto h = disp.subscribe(42, [](int x) {
    TLOG << "Dispatcher got: " << x;
});

// auto h = disp += {42, [](int x) {
//     TLOG << "Dispatcher got: " << x;
// }};
```

Trigger a specific key:

```cpp
std::thread([&] {
    std::this_thread::sleep_for(3s);
    disp(42, 7);      // only callbacks bound to key == 42
}).detach();

h.wait();
```

* A single key may have **multiple subscribers**
* The return value is the **number of callbacks invoked**

---

> [!IMPORTANT]
> ## Queue Waiting Semantics
> **For `EventQueue` and `QueueDispatcher`, `wait()` operates on a per-event basis.**
> This behavior is **different from `Event` and `Dispatcher`** and must be understood correctly.

## EventQueue — Buffered Batch Processing Queue

```cpp
stdpp::event::EventQueue<std::string(std::string, bool)> queue;

auto h = queue.append([](std::string s, bool ok) {
    return ok ? std::move(s) : "";
});
```

Enqueue data (buffered only, no execution):

```cpp
queue.enqueue("Hello", true);
queue.enqueue("Hello", false);
queue.enqueue("Hello", true);
```

Process the queue:

```cpp
std::thread([&] {
    std::this_thread::sleep_for(3s);
    queue();   // process all queued items
}).detach();

h.wait();

auto results = h.last();  // std::vector<std::string>
```

* Each queued item is passed to all callbacks
* `last()` returns the **result set of the most recent batch**

---

## QueueDispatcher — Queued + Key-Based Dispatching

```cpp
stdpp::event::QueueDispatcher<int, int(int)> qd;

auto h = qd.subscribe(1, [](int v) {
    return v * 2;
});
```

Enqueue tasks with keys:

```cpp
qd.enqueue(1, 10);
qd.enqueue(1, 20);
qd.enqueue(1, 30);
```

Process the queue:

```cpp
std::thread([&] {
    std::this_thread::sleep_for(3s);
    qd(1);   // process only the queue for key == 1
}).detach();

h.wait();

auto results = h.last();  // {20, 40, 60}
```

---

## Handle Semantics

Each call to `append()` or `subscribe()` returns a **Handle**:

```cpp
auto h = callback.append(...);
```

The `Handle` provides:

| API         | Description                               |
| ----------- | ----------------------------------------- |
| `wait()`    | Block until the next execution completes  |
| `last()`    | Retrieve the most recent execution result |
| Auto-unbind | Automatically unsubscribed on destruction |
---

## Remove / Unsubscribe

All event types support **explicit removal of callbacks**, allowing manual lifetime control or early unbinding.

---

### Event — Removing Callbacks

```cpp
stdpp::event::Event<int(int)> ev;

auto h = ev.append(&foo);
```

Two removal methods are supported:

#### 1. Remove by Function Pointer

```cpp
ev.remove(&foo);
// ev - &foo;
// ev -= &foo;
```

* Removes **all callbacks** whose target function equals `&foo`
* Suitable for free functions or static member functions

#### 2. Remove by Handle (Recommended)

```cpp
ev.remove(h);
```

* Precisely removes the specific subscription represented by the handle
* Does **not** depend on function identity
* Fully **thread-safe**

> Note:
> `Handle` internally holds a weak reference.
> If the handle is already expired (event destroyed or previously removed),
> `remove(handle)` becomes a no-op.

---

### Dispatcher — Removing Subscriptions

```cpp
stdpp::event::Dispatcher<int, void(int)> disp;
auto h = disp.subscribe(42, &on_event);
```

Supported removal forms:

```cpp
disp.remove(42, &on_event);   // remove this function under key == 42
disp.remove(&on_event);       // remove this function from all keys
disp.remove(42);              // remove all callbacks bound to key == 42
disp.remove(h);               // remove by Handle (precise)
```

---

### EventQueue / QueueDispatcher — Removing Callbacks

```cpp
auto h = queue.append(&process);
queue.remove(&process);
queue.remove(h);
```

```cpp
auto h = qd.subscribe(1, &process);
qd.remove(1, &process);
qd.remove(&process);
qd.remove(1);
qd.remove(h);
```

* Removal only affects **future executions**
* Items already enqueued but not yet processed will be ignored if no callbacks remain

---

### Core Rule

**Each queued item (`enqueue`) produces one waitable completion event per callback.**

That means:

* `enqueue()` → **does not trigger anything**
* `operator()()` processes queued items
* **Each processed item pushes exactly one result per callback**
* `wait()` waits until **at least one result is available**
* `last()` **consumes and clears all currently available results**

---

### Example: Per-Event Waiting

```cpp
stdpp::event::EventQueue<int(int)> q;

auto h = q.append([](int v) {
    return v * 2;
});

q.enqueue(1);
q.enqueue(2);
q.enqueue(3);

std::thread([&] {
    q();   // processes 3 queued events
}).detach();
```

#### Correct Waiting Pattern

```cpp
for (int i = 0; i < 3; ++i) {
    h.wait();           // waits for one event to complete
    auto r = h.last();  // consumes available results
}
```

---

### Common Misconception
```cpp
h.wait();
auto all = h.last();   // does NOT guarantee the entire queue is done
```

Reason:

* `wait()` unblocks as soon as **any result becomes available**
* The first processed event is sufficient to wake it
* Remaining events may still be running

---

### Correct Mental Model

| Component         | Meaning of `wait()`                          |
| ----------------- | -------------------------------------------- |
| `Event`           | Waits for the **next full invocation**       |
| `Dispatcher`      | Waits until **this subscriber is triggered** |
| `EventQueue`      | Waits for **one queued event to complete**   |
| `QueueDispatcher` | Waits for **one queued event under a key**   |


