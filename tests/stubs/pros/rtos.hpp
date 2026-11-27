#pragma once

#include "pros/detail/scheduler.hpp"

#include <cstdint>
#include <functional>

namespace pros {

using detail::Mutex;
using detail::Task;

inline constexpr std::uint32_t E_TASK_STATE_RUNNING = detail::E_TASK_STATE_RUNNING;
inline constexpr std::uint32_t E_TASK_STATE_BLOCKED = detail::E_TASK_STATE_BLOCKED;
inline constexpr std::uint32_t E_TASK_STATE_DELETED = detail::E_TASK_STATE_DELETED;
inline constexpr std::uint32_t E_TASK_STATE_INVALID = detail::E_TASK_STATE_INVALID;

inline void delay(std::uint32_t milliseconds) { detail::Scheduler::instance().delay(milliseconds); }

inline std::uint32_t millis() { return detail::Scheduler::instance().millis(); }

namespace testing {

inline void resetClock() { detail::Scheduler::instance().resetClock(); }

inline void setDelayHook(std::function<void(std::uint32_t)> hook) {
    detail::Scheduler::instance().setHook(std::move(hook));
}

} // namespace testing

} // namespace pros
