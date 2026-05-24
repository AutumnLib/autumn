#include "autumn/motion/motion_common.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>

namespace autumn {

namespace {
std::atomic<bool> g_motionCancelled{false};
}

bool motionCancelled() { return g_motionCancelled.load(std::memory_order_acquire); }

void setMotionCancelled(bool cancelled) { g_motionCancelled.store(cancelled, std::memory_order_release); }

void tankDrive(pros::MotorGroup& left, pros::MotorGroup& right, double leftVoltage, double rightVoltage) {
    left.move_voltage(leftVoltage);
    right.move_voltage(rightVoltage);
}

void arcadeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double forward, double turn, double maxVoltage) {
    const double leftOut = std::clamp(forward + turn, -maxVoltage, maxVoltage);
    const double rightOut = std::clamp(forward - turn, -maxVoltage, maxVoltage);
    tankDrive(left, right, leftOut, rightOut);
}

} // namespace autumn
