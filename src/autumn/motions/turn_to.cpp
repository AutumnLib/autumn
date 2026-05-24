#include "autumn/control/pid.hpp"
#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/turn_to.hpp"
#include "autumn/util/filters.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

namespace {

bool shouldExitMotion(const MotionSettings& settings) {
    return motionCancelled() || pros::Task::notify_take(true, 0) != 0;
}

double pickTurnDirection(TurnDirection direction, Angle error) {
    switch (direction) {
        case TurnDirection::Clockwise:
            return error.value() >= 0 ? 1.0 : -1.0;
        case TurnDirection::CounterClockwise:
            return error.value() >= 0 ? -1.0 : 1.0;
        case TurnDirection::Auto:
        default:
            return error.value() >= 0 ? 1.0 : -1.0;
    }
}

} // namespace

void turnToHeading(Angle target, Time timeout, TurnToParams params, MotionSettings settings) {
    if (settings.leftMotors == nullptr || settings.rightMotors == nullptr || settings.poseGetter == nullptr) {
        return;
    }

    PID angular = settings.angularPid != nullptr ? *settings.angularPid : PID(3, 0, 20);
    angular.reset();
    SlewRateLimiter angularSlew(settings.angularSlew);

    const std::uint32_t start = pros::millis();
    double prevAngular = 0.0;

    while (!shouldExitMotion(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        Angle error = shortestAngleError(target, pose.theta);
        if (std::abs(to_degrees(error)) <= to_degrees(params.earlyExitRange)) {
            break;
        }

        const double direction = pickTurnDirection(params.direction, error);
        const double angularOut = angularSlew.filter(
            std::clamp(angular.update(to_degrees(error)) * direction, -settings.maxVoltage * params.maxSpeed.value(),
                       settings.maxVoltage * params.maxSpeed.value()),
            prevAngular);
        prevAngular = angularOut;
        tankDrive(*settings.leftMotors, *settings.rightMotors, angularOut, -angularOut);
        pros::delay(10);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

void turnToPoint(Pose target, Time timeout, TurnToParams params, MotionSettings settings) {
    const Pose pose = settings.poseGetter();
    turnToHeading(pose.angleTo(target), timeout, params, settings);
}

} // namespace autumn
