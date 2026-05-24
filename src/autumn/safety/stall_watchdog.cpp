#include "autumn/safety/stall_watchdog.hpp"

#include "autumn/motion/motion_handler.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

StallWatchdog::StallWatchdog(pros::MotorGroup& left, pros::MotorGroup& right, std::function<Pose()> poseGetter,
                             Time timeout)
    : left_(left), right_(right), poseGetter_(std::move(poseGetter)), timeout_(timeout) {}

void StallWatchdog::start() {
    if (running_) {
        return;
    }
    running_ = true;
    tripped_ = false;
    task_ = new pros::Task([this] { loop(); });
}

void StallWatchdog::stop() {
    running_ = false;
    if (task_ != nullptr) {
        pros::delay(static_cast<std::uint32_t>(to_milliseconds(timeout_)) + 5);
        delete task_;
        task_ = nullptr;
    }
}

void StallWatchdog::loop() {
    Pose lastPose = poseGetter_ ? poseGetter_() : Pose{};
    std::uint32_t lastMove = pros::millis();

    while (running_) {
        if (!motion_handler::isMoving()) {
            lastMove = pros::millis();
            if (poseGetter_) {
                lastPose = poseGetter_();
            }
            pros::delay(20);
            continue;
        }

        const bool motorsActive = std::abs(left_.get_actual_velocity()) > 0.5 || std::abs(right_.get_actual_velocity()) > 0.5;
        if (poseGetter_) {
            const Pose current = poseGetter_();
            if (current.distance(lastPose) > 0.25_in) {
                lastPose = current;
                lastMove = pros::millis();
            }
        }

        if (motorsActive && pros::millis() - lastMove > static_cast<std::uint32_t>(to_milliseconds(timeout_))) {
            tripped_ = true;
            motion_handler::cancel();
            tankDrive(left_, right_, 0, 0);
            break;
        }
        pros::delay(20);
    }
}

} // namespace autumn
