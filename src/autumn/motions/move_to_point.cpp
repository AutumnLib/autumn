#include "autumn/control/pid.hpp"
#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/move_to_point.hpp"
#include "autumn/util/filters.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

namespace {

bool shouldExitMotion(const MotionSettings& settings) {
    return motionCancelled() || pros::Task::notify_take(true, 0) != 0;
}

double clampOutput(double value, double max) {
    return std::clamp(value, -max, max);
}

} // namespace

void moveToPoint(Pose target, Time timeout, MoveToPointParams params, MotionSettings settings) {
    if (settings.leftMotors == nullptr || settings.rightMotors == nullptr || settings.poseGetter == nullptr) {
        return;
    }

    PID lateral = settings.lateralPid != nullptr ? *settings.lateralPid : PID(10, 0, 3);
    PID angular = settings.angularPid != nullptr ? *settings.angularPid : PID(3, 0, 20);
    lateral.reset();
    angular.reset();

    SlewRateLimiter lateralSlew(settings.lateralSlew);
    SlewRateLimiter angularSlew(settings.angularSlew);

    const std::uint32_t start = pros::millis();
    double prevLateral = 0.0;
    double prevAngular = 0.0;

    while (!shouldExitMotion(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        const Length distance = pose.distance(target);
        if (distance <= params.earlyExitRange) {
            break;
        }

        const Angle angleToTarget = pose.angleTo(target);
        const double direction = params.reversed ? -1.0 : 1.0;
        const double lateralOut = lateralSlew.filter(
            clampOutput(lateral.update(to_inches(distance) * direction), settings.maxVoltage * params.maxLateralSpeed.value()),
            prevLateral);
        const double angularOut = angularSlew.filter(
            clampOutput(angular.update(to_degrees(shortestAngleError(angleToTarget, pose.theta))),
            settings.maxVoltage * params.maxAngularSpeed.value()),
            prevAngular);

        prevLateral = lateralOut;
        prevAngular = angularOut;
        arcadeDrive(*settings.leftMotors, *settings.rightMotors, lateralOut, angularOut, settings.maxVoltage);
        pros::delay(10);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

} // namespace autumn
