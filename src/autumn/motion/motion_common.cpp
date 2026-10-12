#include "autumn/motion/motion_common.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace autumn {

namespace {
std::atomic<bool> g_motionCancelled{false};
}

bool motionCancelled() { return g_motionCancelled.load(std::memory_order_acquire); }

void setMotionCancelled(bool cancelled) { g_motionCancelled.store(cancelled, std::memory_order_release); }

namespace {

std::int32_t toMillivolts(double command) {
    const double clamped = std::clamp(command, -12000.0, 12000.0);
    return static_cast<std::int32_t>(std::lround(clamped));
}

} // namespace

void tankDrive(pros::MotorGroup& left, pros::MotorGroup& right, double leftVoltage, double rightVoltage) {
    left.move_voltage(toMillivolts(leftVoltage));
    right.move_voltage(toMillivolts(rightVoltage));
}

void driveScaled(pros::MotorGroup& left, pros::MotorGroup& right, double lateral, double angular, double maxVoltage) {
    const double scale = std::max(std::abs(maxVoltage), 1e-6);
    double leftOut = lateral + angular;
    double rightOut = lateral - angular;
    const double peak = std::max(std::abs(leftOut), std::abs(rightOut));
    if (peak > scale) {
        leftOut *= scale / peak;
        rightOut *= scale / peak;
    }
    tankDrive(left, right, leftOut / scale * 12000.0, rightOut / scale * 12000.0);
}

void arcadeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double forward, double turn, double maxVoltage) {
    driveScaled(left, right, forward, turn, maxVoltage);
}

} // namespace autumn
