#include "pros/detail/scheduler.hpp"

#include <utility>

namespace pros::detail {

namespace {
thread_local std::shared_ptr<TaskImpl> currentTask;
}

Scheduler& Scheduler::instance() {
    static Scheduler scheduler;
    return scheduler;
}

std::uint32_t Scheduler::millis() const {
    std::lock_guard lock(mu_);
    return now_;
}

void Scheduler::resetClock() {
    std::lock_guard lock(mu_);
    now_ = 0;
}

void Scheduler::setHook(std::function<void(std::uint32_t)> hook) {
    std::lock_guard lock(mu_);
    hook_ = std::move(hook);
}

void Scheduler::wake() {
    std::lock_guard lock(mu_);
    cv_.notify_all();
}

bool Scheduler::settledLocked() {
    bool settled = true;
    std::vector<std::weak_ptr<TaskImpl>> live;
    live.reserve(tasks_.size());
    for (const auto& weak : tasks_) {
        const std::shared_ptr<TaskImpl> task = weak.lock();
        if (!task) {
            continue;
        }
        live.push_back(weak);
        const std::uint32_t state = task->state.load();
        if (state == E_TASK_STATE_STARTING || state == E_TASK_STATE_RUNNING) {
            settled = false;
            continue;
        }
        // A blocked task whose deadline has passed still owes a slice for this epoch.
        if (state == E_TASK_STATE_BLOCKED && now_ >= task->wakeAt && task->blockedEpoch != epoch_) {
            settled = false;
        }
    }
    tasks_.swap(live);
    return settled;
}

void Scheduler::runTask(const std::shared_ptr<TaskImpl>& impl) {
    currentTask = impl;
    try {
        {
            std::lock_guard lock(mu_);
            impl->state = E_TASK_STATE_RUNNING;
        }
        impl->fn();
    } catch (const TaskCancelled&) {
    }
    {
        std::lock_guard lock(mu_);
        impl->state = E_TASK_STATE_DELETED;
        cv_.notify_all();
    }
    currentTask.reset();
}

std::shared_ptr<TaskImpl> Scheduler::spawn(std::function<void()> fn) {
    auto impl = std::make_shared<TaskImpl>();
    impl->fn = std::move(fn);
    {
        std::lock_guard lock(mu_);
        tasks_.push_back(impl);
    }
    impl->thread = std::thread([this, impl] { runTask(impl); });

    std::unique_lock lock(mu_);
    cv_.wait(lock, [&] {
        const std::uint32_t state = impl->state.load();
        return state == E_TASK_STATE_BLOCKED || state == E_TASK_STATE_DELETED;
    });
    return impl;
}

void Scheduler::delayFromTask(const std::shared_ptr<TaskImpl>& self, std::uint32_t milliseconds) {
    std::unique_lock lock(mu_);
    if (self->killed.load()) {
        self->state = E_TASK_STATE_DELETED;
        cv_.notify_all();
        throw TaskCancelled();
    }
    self->wakeAt = now_ + milliseconds;
    self->state = E_TASK_STATE_BLOCKED;
    self->blockedEpoch = epoch_;
    cv_.notify_all();
    cv_.wait(lock, [&] { return self->killed.load() || now_ >= self->wakeAt; });
    if (self->killed.load()) {
        self->state = E_TASK_STATE_DELETED;
        cv_.notify_all();
        throw TaskCancelled();
    }
    self->state = E_TASK_STATE_RUNNING;
}

void Scheduler::delay(std::uint32_t milliseconds) {
    if (const std::shared_ptr<TaskImpl> self = currentTask) {
        delayFromTask(self, milliseconds);
        return;
    }

    std::function<void(std::uint32_t)> hook;
    {
        std::lock_guard lock(mu_);
        now_ += milliseconds;
        hook = hook_;
    }
    if (hook) {
        hook(milliseconds);
    }
    std::unique_lock lock(mu_);
    ++epoch_;
    cv_.notify_all();
    cv_.wait(lock, [&] { return settledLocked(); });
}

void Scheduler::delayUntil(std::uint32_t* previous, std::uint32_t increment) {
    const std::uint32_t target = *previous + increment;
    const std::uint32_t now = millis();
    if (target > now) {
        delay(target - now);
    }
    *previous = target;
}

std::uint32_t Scheduler::notifyTake(bool clearOnExit) {
    const std::shared_ptr<TaskImpl> self = currentTask;
    if (!self) {
        return 0;
    }
    const std::uint32_t value = self->notifyValue.load();
    if (value != 0 && clearOnExit) {
        self->notifyValue.store(0);
    }
    return value;
}

bool Mutex::take(std::uint32_t) {
    mutex_.lock();
    return true;
}

void Mutex::give() { mutex_.unlock(); }

Task& Task::operator=(Task&& other) noexcept {
    if (this != &other) {
        reset();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

Task::~Task() { reset(); }

void Task::reset() {
    if (!impl_) {
        return;
    }
    std::shared_ptr<TaskImpl> impl = std::move(impl_);
    impl->killed.store(true);
    Scheduler::instance().wake();
    if (impl->thread.joinable() && impl->thread.get_id() != std::this_thread::get_id()) {
        impl->thread.join();
    }
}

std::uint32_t Task::get_state() const {
    if (!impl_) {
        return E_TASK_STATE_INVALID;
    }
    return impl_->state.load();
}

std::uint32_t Task::notify() {
    if (!impl_) {
        return 0;
    }
    impl_->notifyValue.store(1);
    return 1;
}

void Task::delay_until(std::uint32_t* previous, std::uint32_t increment) {
    Scheduler::instance().delayUntil(previous, increment);
}

std::uint32_t Task::notify_take(bool clearOnExit, std::uint32_t) {
    return Scheduler::instance().notifyTake(clearOnExit);
}

} // namespace pros::detail
