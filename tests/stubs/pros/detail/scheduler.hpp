#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace pros::detail {

inline constexpr std::uint32_t E_TASK_STATE_RUNNING = 0;
inline constexpr std::uint32_t E_TASK_STATE_STARTING = 1;
inline constexpr std::uint32_t E_TASK_STATE_BLOCKED = 2;
inline constexpr std::uint32_t E_TASK_STATE_DELETED = 3;
inline constexpr std::uint32_t E_TASK_STATE_INVALID = 4;

struct TaskCancelled : std::exception {
    const char* what() const noexcept override { return "pros task cancelled"; }
};

struct TaskImpl {
    std::function<void()> fn;
    std::thread thread;
    std::atomic<std::uint32_t> state{E_TASK_STATE_STARTING};
    std::atomic<bool> killed{false};
    std::atomic<std::uint32_t> notifyValue{0};
    std::uint32_t wakeAt = 0;
    std::uint32_t blockedEpoch = 0;
};

class Scheduler {
public:
    static Scheduler& instance();

    std::uint32_t millis() const;
    void delay(std::uint32_t milliseconds);
    void delayUntil(std::uint32_t* previous, std::uint32_t increment);
    void resetClock();
    void setHook(std::function<void(std::uint32_t)> hook);
    void wake();

    std::shared_ptr<TaskImpl> spawn(std::function<void()> fn);
    std::uint32_t notifyTake(bool clearOnExit);

private:
    Scheduler() = default;

    bool settledLocked();
    void delayFromTask(const std::shared_ptr<TaskImpl>& self, std::uint32_t milliseconds);
    void runTask(const std::shared_ptr<TaskImpl>& impl);

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::uint32_t now_ = 0;
    std::uint32_t epoch_ = 0;
    std::function<void(std::uint32_t)> hook_;
    std::vector<std::weak_ptr<TaskImpl>> tasks_;
};

class Mutex {
public:
    bool take(std::uint32_t timeout = 0xFFFFFFFF);
    void give();

private:
    mutable std::mutex mutex_;
};

class Task {
public:
    template <typename Fn>
    explicit Task(Fn&& fn) : impl_(Scheduler::instance().spawn(std::function<void()>(std::forward<Fn>(fn)))) {}

    Task(Task&& other) noexcept : impl_(std::move(other.impl_)) {}
    Task& operator=(Task&& other) noexcept;
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    ~Task();

    std::uint32_t get_state() const;
    std::uint32_t notify();

    static void delay_until(std::uint32_t* previous, std::uint32_t increment);
    static std::uint32_t notify_take(bool clearOnExit, std::uint32_t timeout);

private:
    void reset();
    std::shared_ptr<TaskImpl> impl_;
};

} // namespace pros::detail
