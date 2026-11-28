#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/motion_math.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace autumn {

namespace {
std::atomic<bool> g_motionCancelled{false};
std::atomic<double> g_motionPathDistance{0.0};
std::atomic<double> g_motionTurnDegrees{0.0};
std::atomic<bool> g_motionProgressPrimed{false};
double g_progressLastX = 0.0;
double g_progressLastY = 0.0;
double g_progressLastThetaDeg = 0.0;
} // namespace

bool motionCancelled() { return g_motionCancelled.load(std::memory_order_acquire); }

void setMotionCancelled(bool cancelled) { g_motionCancelled.store(cancelled, std::memory_order_release); }

void resetMotionProgress() {
    g_motionPathDistance.store(0.0, std::memory_order_relaxed);
    g_motionTurnDegrees.store(0.0, std::memory_order_relaxed);
    g_motionProgressPrimed.store(false, std::memory_order_relaxed);
}

void sampleMotionProgress(const Pose& pose) {
    const double x = to_inches(pose.x);
    const double y = to_inches(pose.y);
    const double thetaDeg = to_degrees(pose.theta);
    if (!g_motionProgressPrimed.load(std::memory_order_relaxed)) {
        g_progressLastX = x;
        g_progressLastY = y;
        g_progressLastThetaDeg = thetaDeg;
        g_motionProgressPrimed.store(true, std::memory_order_relaxed);
        return;
    }
    const double step = std::hypot(x - g_progressLastX, y - g_progressLastY);
    g_motionPathDistance.fetch_add(step, std::memory_order_relaxed);
    double turnDelta = std::remainder(thetaDeg - g_progressLastThetaDeg, 360.0);
    if (turnDelta < -180.0) {
        turnDelta += 360.0;
    }
    g_motionTurnDegrees.fetch_add(std::abs(turnDelta), std::memory_order_relaxed);
    g_progressLastX = x;
    g_progressLastY = y;
    g_progressLastThetaDeg = thetaDeg;
}

double motionPathDistance() { return g_motionPathDistance.load(std::memory_order_relaxed); }

double motionTurnDegrees() { return g_motionTurnDegrees.load(std::memory_order_relaxed); }

void StallTracker::arm(std::uint32_t startMs) {
    startMs_ = startMs;
    stallSinceMs_ = 0;
}

bool StallTracker::update(const MotionSettings& settings, std::uint32_t nowMs, const Pose& pose, Pose& lastPose) {
    if (to_milliseconds(settings.stallExitTime) <= 0) {
        return false;
    }
    if (nowMs - startMs_ < static_cast<std::uint32_t>(settings.stallGraceMs)) {
        stallSinceMs_ = 0;
        lastPose = pose;
        return false;
    }
    const double pathStep = to_inches(pose.distance(lastPose));
    const double turnStep = std::abs(headingErrorDeg(to_degrees(pose.theta), to_degrees(lastPose.theta), TurnDirection::Auto));
    lastPose = pose;
    const bool stopped = pathStep * 100.0 < to_inches(settings.stallSpeedThreshold) &&
                         turnStep * 100.0 < to_degrees(settings.stallTurnThreshold);
    if (!stopped) {
        stallSinceMs_ = 0;
        return false;
    }
    if (stallSinceMs_ == 0) {
        stallSinceMs_ = nowMs;
    }
    return nowMs - stallSinceMs_ >= static_cast<std::uint32_t>(to_milliseconds(settings.stallExitTime));
}

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
