#pragma once

// https://github.com/1992724048/stdpp-event
// 1.0.2

#include <type_traits>
#include <vector>
#include <functional>
#include <unordered_map>
#include <queue>
#include <shared_mutex>
#include <optional>
#include <ranges>

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

    template<typename R, typename... Args>
    class FastEvent<R(Args...)> {
    public:
        using Func = R(*)(Args...);

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
                return {idx, s.generation};
            }

            const uint32_t idx = static_cast<uint32_t>(slots.size());
            slots.push_back({f, 0});
            return {idx, 0};
        }

        auto remove(Handle h) -> void {
            if (h.index >= slots.size()) {
                return;
            }
            auto& s = slots[h.index];
            if (s.generation != h.generation) {
                return;
            }

            s.func = nullptr;
            ++s.generation;
            free_list.push_back(h.index);
        }

        auto operator()(Args... args) noexcept -> void {
            for (auto& s : slots) {
                if (s.func) {
                    s.func(args...);
                }
            }
        }

        auto operator+=(const Func func) -> Handle {
            return append(func);
        }

        auto operator+(const Func func) -> Handle {
            return append(func);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto operator-(const Handle& handle) -> void {
            return remove(handle);
        }

        auto size() const noexcept -> size_t {
            return slots.size() - free_list.size();
        }

    private:
        struct Slot {
            Func func;
            uint32_t generation;
        };

        std::vector<Slot> slots;
        std::vector<uint32_t> free_list;
    };

    template<typename T> requires std::is_function_v<T>
    class Event {
        using Func = std::function<T>;
        using Ret = FnRet<T>::Type;

        static constexpr bool is_void = std::is_void_v<Ret>;

        struct Node {
            explicit Node(Func f) : func(std::move(f)) {}

            Func func;
            mutable std::shared_mutex mutex;
            std::condition_variable_any cv;
            std::atomic<uint64_t> seq = 0;
            std::conditional_t<is_void, bool, std::optional<Ret>> last_value{};
        };

    public:
        class Handle {
        public:
            [[nodiscard]] auto last() const {
                if (node.expired()) {
                    return decltype(Node::last_value)();
                }

                auto n = node.lock();
                if (!n) {
                    if constexpr (is_void) {
                        return false;
                    } else {
                        return std::optional<Ret>{};
                    }
                }

                std::shared_lock _(n->mutex);
                if constexpr (is_void) {
                    return n->last_value;
                } else {
                    return n->last_value;
                }
            }

            template<class Rep, class Period>
            [[nodiscard]] auto wait(std::chrono::duration<Rep, Period> timeout) const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                auto start_seq = n->seq.load();
                return n->cv.wait_for(lk,
                                      timeout,
                                      [&] {
                                          return n->seq != start_seq;
                                      });
            }

            [[nodiscard]] auto wait() const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                auto start_seq = n->seq.load();
                n->cv.wait(lk,
                           [&] {
                               return n->seq != start_seq;
                           });
                return true;
            }

        private:
            friend class Event;
            std::weak_ptr<Node> node;
        };

        [[nodiscard]] auto append(T* func) -> Handle {
            std::unique_lock _(mutex);

            auto node = std::make_shared<Node>(Func(func));
            nodes.push_back(node);

            Handle h;
            h.node = node;
            return h;
        }

        auto operator+=(T* func) -> Handle {
            return append(func);
        }

        auto operator+(T* func) -> Handle {
            return append(func);
        }

        auto operator-=(T* func) -> void {
            return remove(func);
        }

        auto operator-(T* func) -> void {
            return remove(func);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto operator-(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(T* func) -> void {
            std::unique_lock _(mutex);
            std::erase_if(nodes,
                          [&](std::shared_ptr<Node> node) {
                              auto p = node->func.template target<T*>();
                              return p && *p == func;
                          });
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node.lock();
            if (!target) {
                return;
            }

            std::unique_lock _(mutex);
            std::erase_if(nodes,
                          [&](const std::shared_ptr<Node>& node) {
                              if (!handle.node.expired()) {
                                  return node == target;
                              }
                              return false;
                          });
        }

        template<typename... Args>
        auto operator()(Args&&... args) -> size_t {
            std::shared_lock _(mutex);
            size_t count{0};
            for (auto& n : nodes) {
                try {
                    if constexpr (is_void) {
                        n->func(std::forward<Args>(args)...);
                        std::unique_lock lk(n->mutex);
                        n->last_value = true;
                    } else {
                        auto r = n->func(std::forward<Args>(args)...);
                        std::unique_lock lk(n->mutex);
                        n->last_value = std::move(r);
                    }
                    count++;
                } catch (...) {
                    std::unique_lock lk(n->mutex);
                    if constexpr (is_void) {
                        n->last_value = false;
                    } else {
                        n->last_value = std::nullopt;
                    }
                }
                ++n->seq;
                n->cv.notify_all();
            }
            return count;
        }

        auto size() const noexcept -> size_t {
            return nodes.size();
        }

    private:
        std::shared_mutex mutex;
        std::vector<std::shared_ptr<Node>> nodes;
    };

    template<typename Key, typename FuncT> requires std::is_function_v<FuncT>
    class Dispatcher {
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;

        static constexpr bool is_void = std::is_void_v<Ret>;

        struct Node {
            explicit Node(Func f) : func(std::move(f)) {}

            Func func;
            mutable std::shared_mutex mutex;
            std::condition_variable_any cv;
            std::atomic<uint64_t> seq = 0;
            std::conditional_t<is_void, bool, std::optional<Ret>> last_value{};
        };

    public:
        class Handle {
        public:
            [[nodiscard]] auto last() const {
                if (node.expired()) {
                    return decltype(Node::last_value)();
                }

                auto n = node.lock();
                if (!n) {
                    if constexpr (is_void) {
                        return false;
                    } else {
                        return std::optional<Ret>{};
                    }
                }

                std::shared_lock _(n->mutex);
                return n->last_value;
            }

            template<class Rep, class Period>
            [[nodiscard]] auto wait(std::chrono::duration<Rep, Period> timeout) const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                auto start_seq = n->seq.load();
                return n->cv.wait_for(lk,
                                      timeout,
                                      [&] {
                                          return n->seq != start_seq;
                                      });
            }

            [[nodiscard]] auto wait() const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                auto start_seq = n->seq.load();
                n->cv.wait(lk,
                           [&] {
                               return n->seq != start_seq;
                           });
                return true;
            }

        private:
            friend class Dispatcher;
            std::weak_ptr<Node> node;
        };

        [[nodiscard]] auto subscribe(const Key& key, FuncT* func) -> Handle {
            auto node = std::make_shared<Node>(std::move(Func(func)));

            {
                std::unique_lock _(mutex);
                dispatchers[key].push_back(node);
            }

            Handle h;
            h.node = node;
            return h;
        }

        [[nodiscard]] auto subscribe(FuncT* func) -> Handle {
            auto node = std::make_shared<Node>(std::move(Func(func)));

            {
                std::unique_lock _(mutex);
                for (auto& dispatcher : dispatchers) {
                    dispatcher.push_back(node);
                }
            }

            Handle h;
            h.node = node;
            return h;
        }

        auto operator+=(std::pair<Key, FuncT*> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator+(std::pair<Key, FuncT*> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator+=(FuncT* func) -> Handle {
            return subscribe(func);
        }

        auto operator+(FuncT* func) -> Handle {
            return subscribe(func);
        }

        auto operator-=(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-=(std::pair<Key, FuncT*> pair) -> void {
            return remove(pair.first, pair.second);
        }

        auto operator-(std::pair<Key, FuncT*> pair) -> void {
            return remove(pair.first, pair.second);
        }

        auto operator-=(Key key) -> void {
            return remove(key);
        }

        auto operator-(Key key) -> void {
            return remove(key);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto operator-(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(const Key& key, FuncT* func) -> void {
            {
                std::unique_lock _(mutex);
                if (!dispatchers.contains(key)) {
                    return;
                }
                std::erase_if(dispatchers[key],
                              [&](std::shared_ptr<Node> node) {
                                  auto p = node->func.template target<FuncT*>();
                                  return p && *p == func;
                              });
            }
            clear_callbacks();
        }

        auto remove(FuncT* func) -> void {
            {
                std::unique_lock _(mutex);
                for (auto& dispatcher : dispatchers | std::views::values) {
                    std::erase_if(dispatcher,
                                  [&](std::shared_ptr<Node> node) {
                                      auto p = node->func.template target<FuncT*>();
                                      return p && *p == func;
                                  });
                }
            }
            clear_callbacks();
        }

        auto remove(const Key& key) -> void {
            {
                std::unique_lock _(mutex);
                if (!dispatchers.contains(key)) {
                    return;
                }
                dispatchers.erase(key);
            }
            clear_callbacks();
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node.lock();
            if (!target) {
                return;
            }

            {
                std::unique_lock _(mutex);
                for (auto& dispatcher : dispatchers | std::views::values) {
                    std::erase_if(dispatcher,
                                  [&](const std::shared_ptr<Node>& node) {
                                      if (!handle.node.expired()) {
                                          return node == target;
                                      }
                                      return false;
                                  });
                }
            }
            clear_callbacks();
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
                        std::unique_lock lk(n->mutex);
                        n->last_value = true;
                    } else {
                        auto r = n->func(std::forward<Args>(args)...);
                        std::unique_lock lk(n->mutex);
                        n->last_value = std::move(r);
                    }
                    count++;
                } catch (...) {
                    std::unique_lock lk(n->mutex);
                    if constexpr (is_void) {
                        n->last_value = false;
                    } else {
                        n->last_value = std::nullopt;
                    }
                }

                ++n->seq;
                n->cv.notify_all();
            }
            return count;
        }

        auto size() const noexcept -> size_t {
            return dispatchers.size();
        }

    private:
        mutable std::shared_mutex mutex;
        std::unordered_map<Key, std::vector<std::shared_ptr<Node>>> dispatchers;

        auto clear_callbacks() -> void {
            std::vector<Key> empty_keys;
            std::shared_lock _(mutex);
            for (const auto& [key, calls] : dispatchers) {
                if (calls.empty()) {
                    empty_keys.push_back(key);
                }
            }

            if (!empty_keys.empty()) {
                for (const auto& key : empty_keys) {
                    if (dispatchers.contains(key)) {
                        dispatchers.erase(key);
                    }
                }
            }
        }
    };

    template<typename FuncT, size_t MaxResults = 1024> requires std::is_function_v<FuncT>
    class EventQueue {
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;
        using ArgsTuple = FnArgs<FuncT>::Tuple;

        static constexpr bool is_void = std::is_void_v<Ret>;

        struct Node {
            explicit Node(Func f) : func(std::move(f)) {}

            Func func;
            mutable std::shared_mutex mutex;
            std::condition_variable_any cv;
            std::conditional_t<is_void, std::queue<bool>, std::queue<std::optional<Ret>>> results;
        };

    public:
        class Handle {
        public:
            using ResultVec = std::conditional_t<is_void, std::vector<bool>, std::vector<std::optional<Ret>>>;

            [[nodiscard]] auto last() -> ResultVec {
                ResultVec out;
                if (node.expired()) {
                    return out;
                }

                auto n = node.lock();
                if (!n) {
                    return out;
                }

                std::unique_lock lk(n->mutex);

                while (!n->results.empty()) {
                    if constexpr (is_void) {
                        out.push_back(n->results.front());
                    } else {
                        out.push_back(std::move(n->results.front()));
                    }
                    n->results.pop();
                }

                return out;
            }

            template<class Rep, class Period>
            [[nodiscard]] auto wait(std::chrono::duration<Rep, Period> timeout) const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                return n->cv.wait_for(lk,
                                      timeout,
                                      [&] {
                                          return !n->results.empty();
                                      });
            }

            [[nodiscard]] auto wait() const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                n->cv.wait(lk,
                           [&] {
                               return !n->results.empty();
                           });
                return true;
            }

        private:
            friend class EventQueue;
            std::weak_ptr<Node> node;
        };

        [[nodiscard]] auto append(FuncT* func) -> Handle {
            auto node = std::make_shared<Node>(Func(func));

            {
                std::unique_lock _(cb_mutex);
                callbacks.push_back(node);
            }

            Handle h;
            h.node = node;
            return h;
        }

        auto operator+=(FuncT* func) -> Handle {
            return append(func);
        }

        auto operator+(FuncT* func) -> Handle {
            return append(func);
        }

        auto operator-=(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto operator-(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(FuncT* func) -> void {
            std::unique_lock _(cb_mutex);
            for (auto& callback : callbacks | std::views::values) {
                std::erase_if(callback,
                              [&](std::shared_ptr<Node> node) {
                                  auto p = node->func.template target<FuncT*>();
                                  return p && *p == func;
                              });
            }
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node.lock();
            if (!target) {
                return;
            }

            std::unique_lock _(cb_mutex);
            std::erase_if(callbacks,
                          [&](const std::shared_ptr<Node>& node) {
                              if (!handle.node.expired()) {
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
                    std::apply([&](auto&&... unpacked) {
                                   try {
                                       std::unique_lock lk(n->mutex);
                                       if (n->results.size() >= MaxResults) {
                                           n->results.pop();
                                       }

                                       if constexpr (is_void) {
                                           n->func(unpacked...);
                                           n->results.push(true);
                                       } else {
                                           auto r = n->func(unpacked...);
                                           n->results.push(std::move(r));
                                       }
                                   } catch (...) {
                                       std::unique_lock lk(n->mutex);
                                       if constexpr (is_void) {
                                           n->results.push(false);
                                       } else {
                                           n->results.push(std::nullopt);
                                       }
                                   }
                                   n->cv.notify_all();
                               },
                               args);
                }

                ++count;
            }

            return count;
        }

        auto size() const noexcept -> size_t {
            return callbacks.size();
        }

        auto queue_size() const noexcept -> size_t {
            return queue.size();
        }

    private:
        std::shared_mutex cb_mutex;
        std::vector<std::shared_ptr<Node>> callbacks;

        std::shared_mutex queue_mutex;
        std::queue<ArgsTuple> queue;
    };

    template<typename Key, typename FuncT, size_t MaxResults = 1024> requires std::is_function_v<FuncT>
    class QueueDispatcher {
        using Func = std::function<FuncT>;
        using Ret = FnRet<FuncT>::Type;
        using ArgsTuple = FnArgs<FuncT>::Tuple;

        static constexpr bool is_void = std::is_void_v<Ret>;

        struct Node {
            explicit Node(Func f) : func(std::move(f)) {}

            Func func;
            mutable std::shared_mutex mutex;
            std::condition_variable_any cv;
            std::conditional_t<is_void, std::queue<bool>, std::queue<std::optional<Ret>>> results;
        };

    public:
        class Handle {
        public:
            using ResultVec = std::conditional_t<is_void, std::vector<bool>, std::vector<std::optional<Ret>>>;

            [[nodiscard]] auto last() -> ResultVec {
                ResultVec out;
                if (node.expired()) {
                    return out;
                }

                auto n = node.lock();
                if (!n) {
                    return out;
                }

                std::unique_lock lk(n->mutex);
                while (!n->results.empty()) {
                    if constexpr (is_void) {
                        out.push_back(n->results.front());
                    } else {
                        out.push_back(std::move(n->results.front()));
                    }
                    n->results.pop();
                }
                return out;
            }

            template<class Rep, class Period>
            [[nodiscard]] auto wait(std::chrono::duration<Rep, Period> timeout) const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                return n->cv.wait_for(lk,
                                      timeout,
                                      [&] {
                                          return !n->results.empty();
                                      });
            }

            [[nodiscard]] auto wait() const -> bool {
                if (node.expired()) {
                    return false;
                }

                auto n = node.lock();
                if (!n) {
                    return false;
                }

                std::unique_lock lk(n->mutex);
                n->cv.wait(lk,
                           [&] {
                               return !n->results.empty();
                           });
                return true;
            }

        private:
            friend class QueueDispatcher;
            std::weak_ptr<Node> node;
        };

        [[nodiscard]] auto subscribe(const Key& key, FuncT* func) -> Handle {
            auto node = std::make_shared<Node>(Func(func));

            {
                std::unique_lock _(cb_mutex);
                callbacks[key].push_back(node);
            }

            Handle h;
            h.node = node;
            return h;
        }

        auto operator+=(std::pair<Key, FuncT*> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator+(std::pair<Key, FuncT*> pair) -> Handle {
            return subscribe(pair.first, pair.second);
        }

        auto operator+=(FuncT* func) -> Handle {
            return subscribe(func);
        }

        auto operator+(FuncT* func) -> Handle {
            return subscribe(func);
        }

        auto operator-=(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-(FuncT* func) -> void {
            return remove(func);
        }

        auto operator-=(std::pair<Key, FuncT*> pair) -> void {
            return remove(pair.first, pair.second);
        }

        auto operator-(std::pair<Key, FuncT*> pair) -> void {
            return remove(pair.first, pair.second);
        }

        auto operator-=(Key key) -> void {
            return remove(key);
        }

        auto operator-(Key key) -> void {
            return remove(key);
        }

        auto operator-=(const Handle& handle) -> void {
            return remove(handle);
        }

        auto operator-(const Handle& handle) -> void {
            return remove(handle);
        }

        auto remove(const Key& key, FuncT* func) -> void {
            {
                std::unique_lock _(cb_mutex);
                if (!callbacks.contains(key)) {
                    return;
                }

                auto& callback_vec = callbacks[key];
                std::erase_if(callback_vec,
                              [&](std::shared_ptr<Node> node) {
                                  auto p = node->func.template target<FuncT*>();
                                  return p && *p == func;
                              });
            }

            clear_callbacks();
        }

        auto remove(FuncT* func) -> void {
            {
                std::unique_lock _(cb_mutex);
                for (auto& dispatcher : callbacks | std::views::values) {
                    std::erase_if(dispatcher,
                                  [&](std::shared_ptr<Node> node) {
                                      auto p = node->func.template target<FuncT*>();
                                      return p && *p == func;
                                  });
                }
            }

            clear_callbacks();
        }

        auto remove(const Key& key) -> void {
            {
                std::unique_lock _(cb_mutex);
                if (!callbacks.contains(key)) {
                    return;
                }
                callbacks.erase(key);
                if (!queues.contains(key)) {
                    return;
                }
                queues.erase(key);
            }
            clear_callbacks();
        }

        auto remove(const Handle& handle) -> void {
            auto target = handle.node.lock();
            if (!target) {
                return;
            }

            {
                std::unique_lock _(cb_mutex);
                for (auto& dispatcher : callbacks | std::views::values) {
                    std::erase_if(dispatcher,
                                  [&](const std::shared_ptr<Node>& node) {
                                      if (!handle.node.expired()) {
                                          return node == target;
                                      }
                                      return false;
                                  });
                }
            }

            clear_callbacks();
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
                    std::apply([&](auto&&... unpacked) {
                                   try {
                                       std::unique_lock lk(n->mutex);
                                       if (n->results.size() >= MaxResults) {
                                           n->results.pop();
                                       }
                                       if constexpr (is_void) {
                                           n->func(unpacked...);
                                           n->results.push(true);
                                       } else {
                                           auto r = n->func(unpacked...);
                                           n->results.push(std::move(r));
                                       }
                                   } catch (...) {
                                       std::unique_lock lk(n->mutex);
                                       if constexpr (is_void) {
                                           n->results.push(false);
                                       } else {
                                           n->results.push(std::nullopt);
                                       }
                                   }
                                   n->cv.notify_all();
                               },
                               args);
                }

                ++processed;
            }

            return processed;
        }

        auto size() const noexcept -> size_t {
            return callbacks.size();
        }

        auto queue_size() const noexcept -> size_t {
            return queues.size();
        }

    private:
        std::shared_mutex cb_mutex;
        std::unordered_map<Key, std::vector<std::shared_ptr<Node>>> callbacks;

        std::shared_mutex queue_mutex;
        std::unordered_map<Key, std::queue<ArgsTuple>> queues;

        auto clear_callbacks() -> void {
            std::vector<Key> empty_keys;
            std::shared_lock cb_lock(cb_mutex);
            for (const auto& [key, calls] : callbacks) {
                if (calls.empty()) {
                    empty_keys.push_back(key);
                }
            }

            if (!empty_keys.empty()) {
                std::unique_lock queue_lock(queue_mutex);
                for (const auto& key : empty_keys) {
                    if (queues.contains(key)) {
                        queues.erase(key);
                    }
                    if (callbacks.contains(key)) {
                        callbacks.erase(key);
                    }
                }
            }
        }
    };
}
