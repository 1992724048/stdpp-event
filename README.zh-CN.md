<p align="right">
  <a href="README.md">English</a> | <a href="README.zh-CN.md">中文</a>
</p>

# stdpp::event

轻量、类型安全、现代化的 C++ 事件工具库，支持回调聚合、按键分发和队列批处理，并提供异步等待能力。

提供 **五种可组合的基础组件**，从零开销的 `FastEvent` 到完全线程安全的 `QueueDispatcher`，所有组件通过统一的 `Handle` API 连接。

## 目录

- [组件一览](#组件一览)
- [FastEvent — 零开销轻量事件](#fastevent--零开销轻量事件)
- [Event — 聚合回调](#event--聚合回调)
- [Dispatcher — 按键分发](#dispatcher--按键分发)
- [EventQueue — 缓冲队列批处理](#eventqueue--缓冲队列批处理)
- [QueueDispatcher — 队列 + 按键分发](#queuedispatcher--队列--按键分发)
- [Handle 语义](#handle-语义)
- [取消订阅](#取消订阅)
- [线程安全](#线程安全)
- [设计说明](#设计说明)

---

## 组件一览

| 组件                               | 分发方式 | 队列       | 返回值收集                 | 线程安全 | 开销                 |
| ---------------------------------- | -------- | ---------- | -------------------------- | -------- | -------------------- |
| `FastEvent<R, Args...>`            | 广播     | —          | ❌                         | ❌       | 零开销（裸函数指针） |
| `Event<R(Args...)>`                | 广播     | —          | `Handle::last()`           | ✅       | 中等                 |
| `Dispatcher<Key, R(Args...)>`      | 按键     | —          | `Handle::last()`           | ✅       | 中等                 |
| `EventQueue<R(Args...)>`           | 广播     | ✅ enqueue | `Handle::last()`（vector） | ✅       | 中等                 |
| `QueueDispatcher<Key, R(Args...)>` | 按键     | ✅ enqueue | `Handle::last()`（vector） | ✅       | 中等                 |

---

## FastEvent — 零开销轻量事件

> 高频、无锁的事件调用，使用**裸函数指针** + **基于 generation 的槽位复用**。

```cpp
stdpp::event::FastEvent<void, int> fast;   // R 与 Args 为分离的模板参数

auto h1 = fast.append([](int v) { /* ... */ });
auto h2 = fast += [](int v) { /* ... */ };

fast(42);          // 触发所有回调
fast.remove(h1);   // 按 Handle 移除
fast -= h2;        // 按 Handle 移除
```

- 使用**裸函数指针**（`R(*)(Args...)`），无 `std::function` 开销
- **Generation-based 槽位管理**，防止 ABA 问题
- `free_list` 回收已移除的槽位
- **非线程安全** — 专为单线程热点路径设计
- 仅返回 `void`（不支持返回值收集）
- `operator()` 为索引遍历：回调内 append 新回调是安全的（新回调在本轮稍后被触发），且每个回调异常被隔离

---

## Event — 聚合回调

> 收集并调用多个回调，每个回调独立持有返回值。

```cpp
stdpp::event::Event<int(int, int)> process;

auto h1 = process.append([](int a, int b) { return a + b; });
auto h2 = process += [](int a, int b) { return a + b + a; };

// 异步调用与结果同步
std::thread([&] {
    std::this_thread::sleep_for(5s);
    process(1, 2);                // 依序调用所有回调
}).detach();

h1.wait();                        // 等待首次执行完成
auto r1 = h1.last();              // std::optional<int>
```

- `operator()` **依序**调用所有回调，返回调用数量
- 每个回调的返回值**独立缓存**在各自的 `Handle::last()` 中
- `Handle::wait()` 阻塞直到下一次执行完成
- 线程安全：调用时在 `shared_lock` 下**快照拷贝**回调列表

---

## Dispatcher — 按键分发

> 按 Key 将事件路由到不同的订阅者。

```cpp
stdpp::event::Dispatcher<int, void(int)> disp;

auto h = disp.subscribe(42, [](int x) {
    // 处理 key 42 的事件
});

disp(42, 7);               // 仅触发 key 42 的订阅者

disp.remove(42);           // 移除 key 42 的所有订阅
disp.remove(h);            // 按 Handle 精确移除
```

- 一个 Key 可以拥有**多个订阅者**
- 支持**全局订阅**（不指定 Key），接收所有 Key 的事件
- `operator()` 返回被调用的回调数量
- `Handle::last()` 和 `Handle::wait()` 用法与 `Event` 一致

---

## EventQueue — 缓冲队列批处理

> 先入队事件，稍后批量处理。

```cpp
stdpp::event::EventQueue<std::string(std::string, bool)> queue;

auto h = queue.append([](std::string s, bool ok) {
    return ok ? std::move(s) : "";
});

queue.enqueue("Hello", true);     // 仅缓冲，不触发执行
queue.enqueue("Hello", false);

std::thread([&] {
    std::this_thread::sleep_for(3s);
    queue();                       // 批量处理所有队列项
}).detach();

h.wait();
auto results = h.last();           // std::vector<std::string>
```

- `enqueue()` 缓冲参数——**不会触发任何执行**
- `operator()` 批量处理队列
- `Handle::last()` 一次性取出所有积压结果
- `MaxResults` 模板参数控制结果队列容量（默认 1024）

---

## QueueDispatcher — 队列 + 按键分发

> 结合按键路由与队列批处理。

```cpp
stdpp::event::QueueDispatcher<int, int(int)> qd;

auto h = qd.subscribe(1, [](int v) { return v * 2; });

qd.enqueue(1, 10);                 // 缓冲到 key 1
qd.enqueue(1, 20);
qd.enqueue(1, 30);

std::thread([&] {
    std::this_thread::sleep_for(3s);
    qd(1);                         // 仅处理 key 1 的队列
}).detach();

h.wait();
auto results = h.last();           // {20, 40, 60}
```

- 结合 `Dispatcher` 的路由能力 + `EventQueue` 的缓冲能力
- 每个 Key 拥有独立的事件队列
- 支持全局订阅（不指定 Key）

---

## Handle 语义

每次调用 `append()` / `subscribe()` 都会返回一个 **Handle**：

```cpp
auto h = event.append(callback);
```

| API                | 说明                            |
| ------------------ | ------------------------------- |
| `wait()`           | 阻塞直到**下一次**执行完成      |
| `wait(timeout)`    | 带超时的阻塞等待（等待下一次）  |
| `wait_until(target)` | 阻塞直到执行计数达到 `target` |
| `seq()`            | 查询当前执行计数                |
| `last()`           | 获取最近一次执行的结果          |
| 自动解绑           | Handle 析构时自动取消订阅       |

**各组件的 `wait()` 语义差异：**

| 组件                             | `wait()` 含义              |
| -------------------------------- | -------------------------- |
| `Event` / `Dispatcher`           | 等待**一次完整调用**完成   |
| `EventQueue` / `QueueDispatcher` | 等待**一个队列项**处理完成 |

> 对于队列组件，`wait()` 在**任意一个**队列项完成后就会返回。如需等待整批完成，需用循环：
>
> ```cpp
> for (int i = 0; i < 3; ++i) {
>     h.wait();
>     auto r = h.last();
> }
> ```

**推荐的可靠等待模式：** `wait()` 总是等待*下一次*执行——如果事件在你调用 `wait()` 之前已经触发，它会阻塞到*下下次*。如需确定性等待某次特定执行，使用 `seq()` + `wait_until()`：

```cpp
const auto base = h.seq();   // 记录当前计数
event(42);                   // 触发
h.wait_until(base + 1);      // 本次触发完成后返回 true
```

**Handle 内部持有 `weak_ptr`** — 不会延长回调节点的生命周期。如果事件被销毁或回调被移除，所有 Handle 操作安全地成为空操作（`seq()` 返回 0，等待返回 `false`）。

---

## 取消订阅

所有组件支持按**函数指针**或按 **Handle** 移除：

```cpp
// 按函数指针（移除所有匹配的回调）
event.remove(&my_func);
event -= &my_func;

// 按 Handle（精确移除，推荐）
event.remove(h);
event -= h;
```

### Dispatcher 特有移除

```cpp
disp.remove(42, &my_func);   // 移除指定 key 下的函数
disp.remove(&my_func);       // 从所有 key 移除该函数
disp.remove(42);              // 移除整个 key
disp.remove(h);               // 按 Handle 移除
```

### 队列组件移除

```cpp
queue.remove(&process);
queue.remove(h);

qd.remove(1, &process);
qd.remove(&process);
qd.remove(1);
qd.remove(h);
```

- 移除仅影响**未来的执行**
- 已入队但尚未处理的项目，若无剩余回调则自动跳过

---

## 线程安全

| 操作                    | 锁                                    | 说明       |
| ----------------------- | ------------------------------------- | ---------- |
| 添加/移除回调           | `unique_lock(mutex)`                  | 独占写     |
| 触发事件 (`operator()`) | `shared_lock(mutex)` + 快照拷贝       | 允许并发读 |
| 读取 `last()`           | `shared_lock(node->mutex)`            | 非阻塞     |
| 写入 `last()`           | `unique_lock(node->mutex)`            | 执行期间   |
| `wait()`                | `unique_lock(node->mutex)` + 条件变量 |            |
| `enqueue()`             | `unique_lock(queue_mutex)`            | 仅队列锁   |

> **异常安全：** `operator()` 对每个回调调用使用 try-catch 包裹。如果某个回调抛出异常，`last_value` 被重置，继续执行下一个回调。

---

## 设计说明

- Header-only C++20，除标准库外无外部依赖
- `Handle` 内部使用 `weak_ptr` — 安全防止悬挂引用
- `FastEvent` 使用裸函数指针 + generation-based 槽位，实现零开销热点路径
- 队列组件提供 `MaxResults` 模板参数（默认 1024），超限时自动淘汰最旧结果
- 回调列表在执行时快照拷贝 — 在 dispatch 过程中添加/移除回调是安全的，下次生效
- `size()` 返回注册回调总数；`queue_size()`（队列组件）返回待处理参数批次数

## 1.3.0 更新内容

- **新增 `seq()` / `wait_until(target)`**（所有 Handle）— 确定性等待特定执行计数（见 [Handle 语义](#handle-语义)）
- **`EventQueue` / `QueueDispatcher` 回调在节点锁外执行** — 回调内对自身句柄调用 `last()` 现在安全（不再自锁死锁）
- **`FastEvent::operator()`** 改为索引遍历并隔离异常 — 抛异常的回调不再导致进程终止
- **`size()` / `queue_size()` 语义统一** — `size()` = 回调总数，`queue_size()` = 待处理参数数（并补锁）
- 修复：`Event::operator-=(FuncT*)`、`Dispatcher` 全局订阅、`EventQueue::remove(FuncT*)`（此前实例化即编译失败）
