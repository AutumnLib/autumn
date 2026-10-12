#include "autumn/motion/motion_math.hpp"
#include "autumn/motion/turn_to.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

namespace {

bool shouldExit(const MotionSettings&) { return motionCancelled() || pros::Task::notify_take(true, 0) != 0; }

void runTurn(Angle target, Time timeout, TurnToParams params, MotionSettings settings, SwingSide* locked) {
    if (settings.leftMotors == nullptr || settings.rightMotors == nullptr || settings.poseGetter == nullptr) {
        return;
    }

    PidController angularFallback = makeAngularFallback(settings);
    ScalarController& angular = resolveAngular(settings, angularFallback);
    angular.reset();

    const double maxOut = settings.maxVoltage * std::max(0.0, params.maxSpeed.value());
    const double minOut = settings.maxVoltage * std::abs(params.minSpeed.value());
    const std::uint32_t start = pros::millis();
    std::uint32_t prev = start;
    double prevPower = 0.0;

    while (!shouldExit(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }
        const Pose pose = settings.poseGetter();
        const double errorDeg = headingErrorDeg(to_degrees(target), to_degrees(pose.theta), params.direction);
        if (std::abs(errorDeg) <= to_degrees(params.earlyExitRange) && to_degrees(params.earlyExitRange) > 0.0) {
            break;
        }
        if (std::abs(errorDeg) < 0.5 && to_degrees(params.earlyExitRange) <= 0.0) {
            break;
        }

        double power = angular.calculate(errorDeg);
        power = std::clamp(power, -maxOut, maxOut);
        const double step = std::max(settings.angularSlew, 0.0);
        power = std::clamp(power, prevPower - step, prevPower + step);
        if (power > 0.0 && power < minOut) {
            power = minOut;
        } else if (power < 0.0 && power > -minOut) {
            power = -minOut;
        }
        prevPower = power;

        if (locked == nullptr) {
            driveScaled(*settings.leftMotors, *settings.rightMotors, 0.0, power, settings.maxVoltage);
        } else if (*locked == SwingSide::Left) {
            driveScaled(*settings.leftMotors, *settings.rightMotors, power / 2.0, power / 2.0, settings.maxVoltage);
        } else {
            driveScaled(*settings.leftMotors, *settings.rightMotors, power / 2.0, -power / 2.0, settings.maxVoltage);
        }

        const std::uint32_t period = 10;
        pros::Task::delay_until(&prev, period);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

} // namespace

void turnToHeading(Angle target, Time timeout, TurnToParams params, MotionSettings settings) {
    runTurn(target, timeout, params, settings, nullptr);
}

void turnToPoint(Pose target, Time timeout, TurnToParams params, MotionSettings settings) {
    if (settings.poseGetter == nullptr) {
        return;
    }
    const Pose pose = settings.poseGetter();
    turnToHeading(pose.angleTo(target), timeout, params, settings);
}

void swingToHeading(Angle target, SwingSide lockedSide, Time timeout, TurnToParams params, MotionSettings settings) {
    runTurn(target, timeout, params, settings, &lockedSide);
}

void swingToPoint(Pose target, SwingSide lockedSide, Time timeout, TurnToParams params, MotionSettings settings) {
    if (settings.poseGetter == nullptr) {
        return;
    }
    const Pose pose = settings.poseGetter();
    swingToHeading(pose.angleTo(target), lockedSide, timeout, params, settings);
}

} // namespace autumn
