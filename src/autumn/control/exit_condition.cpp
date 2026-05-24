#include "autumn/control/exit_condition.hpp"

#include <cmath>

namespace autumn {

ExitCondition::ExitCondition(double tolerance, Time holdTime)
    : tolerance_(std::abs(tolerance)), holdTime_(holdTime) {}

void ExitCondition::setTolerance(double tolerance) { tolerance_ = std::abs(tolerance); }

void ExitCondition::setHoldTime(Time holdTime) { holdTime_ = holdTime; }

bool ExitCondition::update(double error, Time deltaTime) {
    if (exited_) {
        return true;
    }

    if (std::abs(error) <= tolerance_) {
        elapsed_ = elapsed_ + deltaTime;
        if (elapsed_ >= holdTime_) {
            exited_ = true;
        }
    } else {
        elapsed_ = 0_ms;
    }

    return exited_;
}

void ExitCondition::reset() {
    elapsed_ = 0_ms;
    exited_ = false;
}

TimeoutExitCondition::TimeoutExitCondition(Time timeout) : timeout_(timeout) {}

bool TimeoutExitCondition::update(Time deltaTime) {
    if (exited_) {
        return true;
    }

    elapsed_ = elapsed_ + deltaTime;
    if (elapsed_ >= timeout_) {
        exited_ = true;
    }

    return exited_;
}

void TimeoutExitCondition::reset() {
    elapsed_ = 0_ms;
    exited_ = false;
}

} // namespace autumn
