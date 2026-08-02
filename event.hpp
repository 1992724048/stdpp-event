// 2026-08-02 18:33:29

#pragma once

// https://github.com/1992724048/stdpp-event
// 1.3.0

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <ranges>
#include <shared_mutex>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stdpp::event {
    template<typename>
    struct FnRet;

    template<typename R, typename... Args>
    struct FnRet<R(Args...)> {
        using Type = R;
    };

    template<typename>
    struct FnArgs;

    template<typename R, typename... Args>
    struct FnArgs<R(Args...)> {
        using Tuple = std::tuple<std::decay_t<Args>...>;
    };

    template<typename>
    struct EventNode;

    template<typename R, typename... Args>
    struct EventNode<R(Args...)> {
        using Func = std::function<R(Args...)>;
        using Ret = R;
        static constexpr bool is_void = std::is_void_v<Ret>;
        using LastValue = std::conditional_t<is_void, bool, std::optional<Ret>>;

        explicit EventNode(Func f) :
            func(std::move(f)) {}

        Func func;
        mutable std::shared_mutex mutex;
        std::condition_variable_any cv;
        std::atomic<uint64_t> seq = 0;
        LastValue last_value{};
    };

    template<typename>
    struct QueueNode;

    template<typename R, typename... Args>
    struct QueueNode<R(Args...)> {
        using Func = std::function<R(Args...)>;
        using Ret = R;
        static constexpr bool is_void = std::is_void_v<Ret>;
        using ResultQueue = std::conditional_t<is_void, std::queue<bool>, std::queue<std::optional<Ret>>>;

        explicit QueueNode(Func f) :
            func(std::move(f)) {}

        Func func;
        mutable std::shared_mutex mutex;
        std::condition_variable_any cv;
        std::atomic<uint64_t> seq = 0;
        ResultQueue results;
    };

    template<typename NodeT>
    class SeqHandleBase {
    protected:
        std::weak_ptr<NodeT> node_;

        [[nodiscard]] auto lock_node() const -> std::shared_ptr<NodeT> {
            if (node_.expired()) {
                return nullptr;
            }
            return node_.lock();
        }
    public:
        [[nodiscard]] auto seq() const -> uint64_t {
            auto n = lock_node();
            if (!n) {
                return 0;
            }
            return n->seq.load();
        }

        [[nodiscard]] auto wait_until(uint64_t target) const -> bool {
            auto n = lock_node();
            if (!n) {
                return false;
            }
            std::unique_lock lk(n->mutex);
            n->cv.wait(lk,
                       [&] -> auto {
                           return n->seq.load() >= target;
                       });
            return true;
        }

        template<class Rep, class Period>
        [[nodiscard]] auto wait(std::chrono::duration<Rep, Period> timeout) const -> bool {
            auto n = lock_node();
            if (!n) {
                return false;
            }
            std::unique_lock lk(n->mutex);
            const auto start_seq = n->seq.load();
            return n->cv.wait_for(lk,
                                  timeout,
                                  [&] -> auto {
                                      return n->seq.load() >= start_seq + 1;
                                  });
        }

        [[nodiscard]] auto wait() const -> bool {
            auto n = lock_node();
            if (!n) {
                return false;
            }
            std::unique_lock lk(n->mutex);
            const auto start_seq = n->seq.load();
            n->cv.wait(lk,
                       [&] -> auto {
                           return n->seq.load() >= start_seq + 1;
                       });
            return true;
        }
    };

    template<typename NodeT>
    class QueueHandleBase {
    protected:
        std::weak_ptr<NodeT> node_;

        [[nodiscard]] auto lock_node() const -> std::shared_ptr<NodeT> {
            if (node_.expired()) {
                return nullptr;
            }
            return node_.lock();
        }
    public:
        [[nodiscard]] auto seq() const -> uint64_t {
            auto n = lock_node();
            if (!n) {
                return 0;
            }
            return n->seq.load();
        }

        [[nodiscard]] auto wait_until(uint64_t target) const -> bool {
            auto n = lock_node();
            if (!n) {
                return false;
            }
            std::unique_lock lk(n->mutex);
            n->cv.wait(lk,
                       [&] -> auto {
                           return n->seq.load() >= target;
                       });
            return true;
        }

        template<class Rep, class Period>
        [[nodiscard]] auto wait(std::chrono::duration<Rep, Period> timeout) const -> bool {
            auto n = lock_node();
            if (!n) {
                return false;
            }
            std::unique_lock lk(n->mutex);
            const auto start_seq = n->seq.load();
            return n->cv.wait_for(lk,
                                  timeout,
                                  [&] -> auto {
                                      return n->seq.load() >= start_seq + 1;
                                  });
        }

        [[nodiscard]] auto wait() const -> bool {
            auto n = lock_node();
            if (!n) {
                return false;
            }
            std::unique_lock lk(n->mutex);
            const auto start_seq = n->seq.load();
            n->cv.wait(lk,
                       [&] -> auto {
                           return n->seq.load() >= start_seq + 1;
                       });
            return true;
        }
    };

    template<typename R, typename... Args>
    class FastEvent {
    public:
        using Func = R (*)(Args...);

        struct Handle {
            uint32_t index;
            uint32_t generation;
        };

        explicit FastEvent(size_t capacity = 32) {
            slots.reserve(capacity);
            free_list.reserve(capacity);
        }

        auto append(Func f) -> Handle {
            if (!free_list.empty()) {
                uint32_t idx = free_list.back();
                free_list.pop_back();
                auto& s = slots[idx];
                s.func = f;
                return {.index = idx, .generation = s.generation};
            }

            const auto idx = static_cast<uint32_t>(slots.size());
            slots.push_back({f, 0});
            return {.index = idx, .generation = 0};
        }

        auto remove(Handle h) -> void {
            if (h.index >= slots.size()) {
                return;
            }
            auto& s = slots[h.index];
            if (s.generation != h.generation) {
                return;
            }
            if (!s.func) {
                return;
            }

            s.func = nullptr;
            ++s.generation;
            free_list.push_back(h.index);
        }

        auto operator()(Args... args) -> void {
            for (size_t i = 0; i < slots.size(); ++i) {
                auto& s = slots[i];
                if (s.func) {
                    try {
                        s.func(args...);
                    } catch (...) {}
                }
            }
        }

        auto operator+=(const Func func) -> Handle {
            return append(func);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        [[nodiscard]] auto size() const noexcept -> size_t {
            auto sz = slots.size();
            auto fl = free_list.size();
            return sz > fl ? sz - fl : 0;
        }
    private:
        struct Slot {
            Func func;
            uint32_t generation;
        };

        std::vector<Slot> slots;
        std::vector<uint32_t> free_list;
    };

    template<typename FuncT> requires std::is_function_v<FuncT>
    class Event {
    public:
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;
    private:
        using Node = EventNode<FuncT>;
        static constexpr bool is_void = Node::is_void;
    public:
        class Handle : public SeqHandleBase<Node> {
        public:
            [[nodiscard]] auto last() const {
                if (this->node_.expired()) {
                    return typename Node::LastValue{};
                }

                auto n = this->lock_node();
                if (!n) {
                    return typename Node::LastValue{};
                }

                std::shared_lock _(n->mutex);
                return n->last_value;
            }
        private:
            friend class Event;
        };

        [[nodiscard]] auto append(Func func) -> Handle {
            std::unique_lock _(mutex);

            auto node = std::make_shared<Node>(std::move(func));
            nodes.push_back(node);

            Handle h;
            h.node_ = node;
            return h;
        }

        [[nodiscard]] auto append(FuncT* func) -> Handle {
            return append(Func(func));
        }

        auto operator+=(FuncT* func) -> Handle {
            return append(Func(func));
        }

        auto operator+=(Func func) -> Handle {
            return append(func);
        }

        auto operator-=(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(FuncT* func) -> void {
            std::unique_lock _(mutex);
            std::erase_if(nodes,
                          [&](std::shared_ptr<Node> node) -> auto {
                              auto p = node->func.template target<FuncT*>();
                              return p && *p == func;
                          });
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node_.lock();
            if (!target) {
                return;
            }

            std::unique_lock _(mutex);
            std::erase_if(nodes,
                          [&](const std::shared_ptr<Node>& node) -> auto {
                              if (!handle.node_.expired()) {
                                  return node == target;
                              }
                              return false;
                          });
        }

        template<typename... Args>
        auto operator()(Args&&... args) -> size_t {
            std::vector<std::shared_ptr<Node>> targets;
            {
                std::shared_lock _(mutex);
                targets = nodes;
            }

            size_t count{0};
            for (auto& n : targets) {
                try {
                    if constexpr (is_void) {
                        n->func(std::forward<Args>(args)...);
                        std::unique_lock _(n->mutex);
                        n->last_value = true;
                    } else {
                        auto r = n->func(std::forward<Args>(args)...);
                        std::unique_lock _(n->mutex);
                        n->last_value = std::move(r);
                    }
                    count++;
                } catch (...) {
                    std::unique_lock _(n->mutex);
                    n->last_value = {};
                }
                ++n->seq;
                n->cv.notify_all();
            }
            return count;
        }

        [[nodiscard]] auto size() const noexcept -> size_t {
            std::shared_lock _(mutex);
            return nodes.size();
        }
    private:
        mutable std::shared_mutex mutex;
        std::vector<std::shared_ptr<Node>> nodes;
    };

    template<typename Key, typename FuncT> requires std::is_function_v<FuncT>
    class Dispatcher {
    public:
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;
    private:
        using Node = EventNode<FuncT>;
        static constexpr bool is_void = Node::is_void;
    public:
        class Handle : public SeqHandleBase<Node> {
        public:
            [[nodiscard]] auto last() const {
                if (this->node_.expired()) {
                    return typename Node::LastValue{};
                }

                auto n = this->lock_node();
                if (!n) {
                    return typename Node::LastValue{};
                }

                std::shared_lock _(n->mutex);
                return n->last_value;
            }
        private:
            friend class Dispatcher;
        };

        [[nodiscard]] auto subscribe(const Key& key, Func func) -> Handle {
            auto node = std::make_shared<Node>(std::move(func));

            {
                std::unique_lock _(mutex);
                dispatchers[key].push_back(node);
            }

            Handle h;
            h.node_ = node;
            return h;
        }

        [[nodiscard]] auto subscribe(const Key& key, FuncT* func) -> Handle {
            return subscribe(key, Func(func));
        }

        [[nodiscard]] auto subscribe(Func func) -> Handle {
            auto node = std::make_shared<Node>(std::move(func));

            {
                std::unique_lock _(mutex);
                for (auto& dispatcher : dispatchers) {
                    dispatcher.second.push_back(node);
                }
            }

            Handle h;
            h.node_ = node;
            return h;
        }

        [[nodiscard]] auto subscribe(FuncT* func) -> Handle {
            return subscribe(Func(func));
        }

        auto operator+=(std::pair<Key, FuncT*> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator+=(std::pair<Key, Func> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator-=(std::pair<Key, FuncT*> pair) -> void {
            return remove(pair.first, pair.second);
        }

        auto operator-=(Key key) -> void {
            return remove(key);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(const Key& key, FuncT* func) -> void {
            std::unique_lock _(mutex);
            auto it = dispatchers.find(key);
            if (it == dispatchers.end()) {
                return;
            }
            std::erase_if(it->second,
                          [&](std::shared_ptr<Node> node) -> auto {
                              auto p = node->func.template target<FuncT*>();
                              return p && *p == func;
                          });
            if (it->second.empty()) {
                dispatchers.erase(it);
            }
        }

        auto remove(FuncT* func) -> void {
            std::unique_lock _(mutex);
            for (auto it = dispatchers.begin(); it != dispatchers.end();) {
                std::erase_if(it->second,
                              [&](std::shared_ptr<Node> node) -> auto {
                                  auto p = node->func.template target<FuncT*>();
                                  return p && *p == func;
                              });
                if (it->second.empty()) {
                    it = dispatchers.erase(it);
                } else {
                    ++it;
                }
            }
        }

        auto remove(const Key& key) -> void {
            std::unique_lock _(mutex);
            dispatchers.erase(key);
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node_.lock();
            if (!target) {
                return;
            }

            std::unique_lock _(mutex);
            for (auto it = dispatchers.begin(); it != dispatchers.end();) {
                std::erase_if(it->second,
                              [&](const std::shared_ptr<Node>& node) -> auto {
                                  if (!handle.node_.expired()) {
                                      return node == target;
                                  }
                                  return false;
                              });
                if (it->second.empty()) {
                    it = dispatchers.erase(it);
                } else {
                    ++it;
                }
            }
        }

        template<typename... Args>
        auto operator()(const Key& key, Args&&... args) -> size_t {
            std::vector<std::shared_ptr<Node>> targets;
            size_t count{0};

            {
                std::shared_lock _(mutex);
                auto it = dispatchers.find(key);
                if (it == dispatchers.end()) {
                    return count;
                }
                targets = it->second;
            }

            for (auto& n : targets) {
                try {
                    if constexpr (is_void) {
                        n->func(std::forward<Args>(args)...);
                        std::unique_lock _(n->mutex);
                        n->last_value = true;
                    } else {
                        auto r = n->func(std::forward<Args>(args)...);
                        std::unique_lock _(n->mutex);
                        n->last_value = std::move(r);
                    }
                    count++;
                } catch (...) {
                    std::unique_lock _(n->mutex);
                    n->last_value = {};
                }

                ++n->seq;
                n->cv.notify_all();
            }
            return count;
        }

        auto size() const noexcept -> size_t {
            std::shared_lock _(mutex);
            size_t total = 0;
            for (const auto& [key, list] : dispatchers) {
                total += list.size();
            }
            return total;
        }
    private:
        mutable std::shared_mutex mutex;
        std::unordered_map<Key, std::vector<std::shared_ptr<Node>>> dispatchers;
    };

    template<typename FuncT, size_t MaxResults = 1024> requires std::is_function_v<FuncT>
    class EventQueue {
    public:
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;
        using ArgsTuple = FnArgs<FuncT>::Tuple;
    private:
        using Node = QueueNode<FuncT>;
        static constexpr bool is_void = Node::is_void;
    public:
        class Handle : public QueueHandleBase<Node> {
        public:
            using ResultVec = std::conditional_t<is_void, std::vector<bool>, std::vector<std::optional<Ret>>>;

            [[nodiscard]] auto last() -> ResultVec {
                ResultVec out;
                if (this->node_.expired()) {
                    return out;
                }

                auto n = this->lock_node();
                if (!n) {
                    return out;
                }

                std::unique_lock _(n->mutex);
                while (!n->results.empty()) {
                    out.push_back(std::move(n->results.front()));
                    n->results.pop();
                }
                return out;
            }
        private:
            friend class EventQueue;
        };

        [[nodiscard]] auto append(Func func) -> Handle {
            auto node = std::make_shared<Node>(std::move(func));

            {
                std::unique_lock _(cb_mutex);
                callbacks.push_back(node);
            }

            Handle h;
            h.node_ = node;
            return h;
        }

        [[nodiscard]] auto append(FuncT* func) -> Handle {
            return append(Func(func));
        }

        auto operator+=(FuncT* func) -> Handle {
            return append(func);
        }

        auto operator+=(Func func) -> Handle {
            return append(func);
        }

        auto operator-=(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(FuncT* func) -> void {
            std::unique_lock _(cb_mutex);
            std::erase_if(callbacks,
                          [&](const std::shared_ptr<Node>& node) -> auto {
                              auto p = node->func.template target<FuncT*>();
                              return p && *p == func;
                          });
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node_.lock();
            if (!target) {
                return;
            }

            std::unique_lock _(cb_mutex);
            std::erase_if(callbacks,
                          [&](const std::shared_ptr<Node>& node) -> auto {
                              if (!handle.node_.expired()) {
                                  return node == target;
                              }
                              return false;
                          });
        }

        template<typename... Args>
        auto enqueue(Args&&... args) -> void {
            static_assert(std::is_same_v<std::tuple<std::decay_t<Args>...>, ArgsTuple>, "enqueue args must match FuncT signature");

            {
                std::unique_lock _(queue_mutex);
                queue.emplace(std::forward<Args>(args)...);
            }
        }

        auto operator()() -> size_t {
            std::queue<ArgsTuple> local;

            {
                std::unique_lock _(queue_mutex);
                std::swap(local, queue);
            }

            if (local.empty()) {
                return 0;
            }

            std::vector<std::shared_ptr<Node>> targets;
            {
                std::shared_lock _(cb_mutex);
                targets = callbacks;
            }

            size_t count{0};

            while (!local.empty()) {
                auto args = std::move(local.front());
                local.pop();

                for (auto& n : targets) {
                    std::apply([&](auto&&... unpacked) -> auto {
                                   try {
                                       if constexpr (is_void) {
                                           n->func(unpacked...);
                                           std::unique_lock _(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(true);
                                       } else {
                                           auto r = n->func(unpacked...);
                                           std::unique_lock _(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(std::move(r));
                                       }
                                   } catch (...) {
                                       std::unique_lock _(n->mutex);
                                       if (n->results.size() >= MaxResults) {
                                           n->results.pop();
                                       }
                                       n->results.push({});
                                   }
                                   ++n->seq;
                                   n->cv.notify_all();
                               },
                               args);
                }

                ++count;
            }

            return count;
        }

        [[nodiscard]] auto size() const noexcept -> size_t {
            std::shared_lock _(cb_mutex);
            return callbacks.size();
        }

        [[nodiscard]] auto queue_size() const noexcept -> size_t {
            std::shared_lock _(queue_mutex);
            return queue.size();
        }
    private:
        mutable std::shared_mutex cb_mutex;
        std::vector<std::shared_ptr<Node>> callbacks;

        mutable std::shared_mutex queue_mutex;
        std::queue<ArgsTuple> queue;
    };

    template<typename Key, typename FuncT, size_t MaxResults = 1024> requires std::is_function_v<FuncT>
    class QueueDispatcher {
    public:
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;
        using ArgsTuple = FnArgs<FuncT>::Tuple;
    private:
        using Node = QueueNode<FuncT>;
        static constexpr bool is_void = Node::is_void;
    public:
        class Handle : public QueueHandleBase<Node> {
        public:
            using ResultVec = std::conditional_t<is_void, std::vector<bool>, std::vector<std::optional<Ret>>>;

            [[nodiscard]] auto last() -> ResultVec {
                ResultVec out;
                if (this->node_.expired()) {
                    return out;
                }

                auto n = this->lock_node();
                if (!n) {
                    return out;
                }

                std::unique_lock _(n->mutex);
                while (!n->results.empty()) {
                    out.push_back(std::move(n->results.front()));
                    n->results.pop();
                }
                return out;
            }
        private:
            friend class QueueDispatcher;
        };

        [[nodiscard]] auto subscribe(const Key& key, Func func) -> Handle {
            auto node = std::make_shared<Node>(std::move(func));

            {
                std::unique_lock _(cb_mutex);
                callbacks[key].push_back(node);
            }

            Handle h;
            h.node_ = node;
            return h;
        }

        [[nodiscard]] auto subscribe(const Key& key, FuncT* func) -> Handle {
            return subscribe(key, Func(func));
        }

        auto operator+=(std::pair<Key, FuncT*> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator+=(std::pair<Key, Func> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator-=(std::pair<Key, FuncT*> pair) -> void {
            return remove(pair.first, pair.second);
        }

        auto remove(const Key& key, FuncT* func) -> void {
            std::unique_lock _(cb_mutex);
            auto it = callbacks.find(key);
            if (it == callbacks.end()) {
                return;
            }
            std::erase_if(it->second,
                          [&](std::shared_ptr<Node> node) -> auto {
                              auto p = node->func.template target<FuncT*>();
                              return p && *p == func;
                          });
            if (it->second.empty()) {
                queues.erase(key);
                callbacks.erase(it);
            }
        }

        auto remove(FuncT* func) -> void {
            std::unique_lock _(cb_mutex);
            for (auto it = callbacks.begin(); it != callbacks.end();) {
                std::erase_if(it->second,
                              [&](std::shared_ptr<Node> node) -> auto {
                                  auto p = node->func.template target<FuncT*>();
                                  return p && *p == func;
                              });
                if (it->second.empty()) {
                    queues.erase(it->first);
                    it = callbacks.erase(it);
                } else {
                    ++it;
                }
            }
        }

        auto remove(const Key& key) -> void {
            std::unique_lock _(cb_mutex);
            callbacks.erase(key);
            queues.erase(key);
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node_.lock();
            if (!target) {
                return;
            }

            std::unique_lock _(cb_mutex);
            for (auto it = callbacks.begin(); it != callbacks.end();) {
                std::erase_if(it->second,
                              [&](const std::shared_ptr<Node>& node) -> auto {
                                  if (!handle.node_.expired()) {
                                      return node == target;
                                  }
                                  return false;
                              });
                if (it->second.empty()) {
                    queues.erase(it->first);
                    it = callbacks.erase(it);
                } else {
                    ++it;
                }
            }
        }

        template<typename... Args>
        auto enqueue(const Key& key, Args&&... args) -> void {
            static_assert(std::is_same_v<std::tuple<std::decay_t<Args>...>, ArgsTuple>, "enqueue args must match FuncT signature");

            std::unique_lock _(cb_mutex);
            if (callbacks.contains(key)) {
                std::unique_lock _(queue_mutex);
                queues[key].emplace(std::forward<Args>(args)...);
            }
        }

        auto operator()(const Key& key) -> size_t {
            std::queue<ArgsTuple> local;

            {
                std::unique_lock _(queue_mutex);
                auto it = queues.find(key);
                if (it == queues.end()) {
                    return 0;
                }
                std::swap(local, it->second);
                queues.erase(key);
            }

            std::vector<std::shared_ptr<Node>> targets;
            {
                std::shared_lock _(cb_mutex);
                auto it = callbacks.find(key);
                if (it == callbacks.end()) {
                    return 0;
                }
                targets = it->second;
            }

            size_t processed = 0;
            while (!local.empty()) {
                auto args = std::move(local.front());
                local.pop();

                for (auto& n : targets) {
                    std::apply([&](auto&&... unpacked) -> auto {
                                   try {
                                       if constexpr (is_void) {
                                           n->func(unpacked...);
                                           std::unique_lock _(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(true);
                                       } else {
                                           auto r = n->func(unpacked...);
                                           std::unique_lock _(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(std::move(r));
                                       }
                                   } catch (...) {
                                       std::unique_lock _(n->mutex);
                                       if (n->results.size() >= MaxResults) {
                                           n->results.pop();
                                       }
                                       n->results.push({});
                                   }
                                   ++n->seq;
                                   n->cv.notify_all();
                               },
                               args);
                }

                ++processed;
            }

            return processed;
        }

        [[nodiscard]] auto size() const noexcept -> size_t {
            std::shared_lock _(cb_mutex);
            size_t total = 0;
            for (const auto& [key, list] : callbacks) {
                total += list.size();
            }
            return total;
        }

        [[nodiscard]] auto queue_size() const noexcept -> size_t {
            std::shared_lock _(queue_mutex);
            size_t total = 0;
            for (const auto& [key, q] : queues) {
                total += q.size();
            }
            return total;
        }
    private:
        mutable std::shared_mutex cb_mutex;
        std::unordered_map<Key, std::vector<std::shared_ptr<Node>>> callbacks;

        mutable std::shared_mutex queue_mutex;
        std::unordered_map<Key, std::queue<ArgsTuple>> queues;
    };
} // namespace stdpp::event
