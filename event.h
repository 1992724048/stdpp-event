#pragma once

#include <type_traits>
#include <vector>
#include <functional>
#include <unordered_map>
#include <queue>
#include <shared_mutex>
#include <optional>

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

    private:
        mutable std::shared_mutex mutex;
        std::unordered_map<Key, std::vector<std::shared_ptr<Node>>> dispatchers;
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
            std::conditional_t<is_void, std::queue<bool>, std::queue<Ret>> results;
        };

    public:
        class Handle {
        public:
            using ResultVec = std::conditional_t<is_void, std::vector<bool>, std::vector<Ret>>;

            [[nodiscard]] auto last() -> ResultVec {
                ResultVec out;

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
                                       if constexpr (is_void) {
                                           n->func(unpacked...);
                                           std::unique_lock lk(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(true);
                                       } else {
                                           auto r = n->func(unpacked...);
                                           std::unique_lock lk(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(std::move(r));
                                       }
                                   } catch (...) {
                                       std::unique_lock lk(n->mutex);
                                       if constexpr (is_void) {
                                           n->results.push(false);
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
            std::conditional_t<is_void, std::queue<bool>, std::queue<Ret>> results;
        };

    public:
        class Handle {
        public:
            using ResultVec = std::conditional_t<is_void, std::vector<bool>, std::vector<Ret>>;

            [[nodiscard]] auto last() -> ResultVec {
                ResultVec out;

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

        template<typename... Args>
        auto enqueue(const Key& key, Args&&... args) -> void {
            static_assert(std::is_same_v<std::tuple<std::decay_t<Args>...>, ArgsTuple>, "enqueue args must match FuncT signature");

            std::unique_lock _(queue_mutex);
            queues[key].emplace(std::forward<Args>(args)...);
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
                                       if constexpr (is_void) {
                                           n->func(unpacked...);
                                           std::unique_lock lk(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(true);
                                       } else {
                                           auto r = n->func(unpacked...);
                                           std::unique_lock lk(n->mutex);
                                           if (n->results.size() >= MaxResults) {
                                               n->results.pop();
                                           }
                                           n->results.push(std::move(r));
                                       }
                                   } catch (...) {
                                       std::unique_lock lk(n->mutex);
                                       if constexpr (is_void) {
                                           n->results.push(false);
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

    private:
        std::shared_mutex cb_mutex;
        std::unordered_map<Key, std::vector<std::shared_ptr<Node>>> callbacks;

        std::shared_mutex queue_mutex;
        std::unordered_map<Key, std::queue<ArgsTuple>> queues;
    };
}
