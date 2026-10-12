#include "autumn/motion/motion_math.hpp"
#include "autumn/motion/move_to_point.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace autumn {

namespace {

bool shouldExit(const MotionSettings&) { return motionCancelled() || pros::Task::notify_take(true, 0) != 0; }

} // namespace

void moveToPoint(Pose target, Time timeout, MoveToPointParams params, MotionSettings settings) {
    if (settings.leftMotors == nullptr || settings.rightMotors == nullptr || settings.poseGetter == nullptr) {
        return;
    }

    PidController lateralFallback = makeLateralFallback(settings);
    PidController angularFallback = makeAngularFallback(settings);
    ScalarController& lateral = resolveLateral(settings, lateralFallback);
    ScalarController& angular = resolveAngular(settings, angularFallback);
    lateral.reset();
    angular.reset();

    const double maxLat = settings.maxVoltage * std::max(0.0, params.maxLateralSpeed.value());
    const double maxAng = settings.maxVoltage * std::max(0.0, params.maxAngularSpeed.value());
    const double minLat = settings.maxVoltage * std::abs(params.minLateralSpeed.value());
    const std::uint32_t start = pros::millis();
    std::uint32_t prevTick = start;
    double prevLateral = 0.0;
    double prevAngular = 0.0;
    bool close = false;
    std::optional<bool> prevSide;

    const Pose initial = settings.poseGetter();
    target.theta = initial.angleTo(target);

    while (!shouldExit(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        const double distance = to_inches(pose.distance(target));
        if (distance <= to_inches(params.earlyExitRange) && to_inches(params.earlyExitRange) > 0.0) {
            break;
        }
        if (!close && distance < 7.5) {
            close = true;
        }

        const bool side = crossedPerpendicular(pose, target, target.theta, params.earlyExitRange);
        if (!prevSide.has_value()) {
            prevSide = side;
        }
        if (minLat > 0.0 && prevSide.has_value() && side != *prevSide) {
            break;
        }
        prevSide = side;

        const double bearingDeg = to_degrees(pose.angleTo(target));
        double faceDeg = bearingDeg;
        const double noseError = headingErrorDeg(bearingDeg, to_degrees(pose.theta), TurnDirection::Auto);
        // Target behind the nose: back up instead of turning around.
        if (!params.reversed && std::cos(noseError * M_PI / 180.0) < 0.0) {
            faceDeg = bearingDeg + 180.0;
        }
        const double robotDeg = to_degrees(pose.theta) + (params.reversed ? 180.0 : 0.0);
        const double angularError = headingErrorDeg(faceDeg, robotDeg, TurnDirection::Auto);
        double lateralError = distance * std::cos(noseError * M_PI / 180.0);

        double lateralOut = lateral.calculate(lateralError);
        double angularOut = close ? 0.0 : angular.calculate(angularError);
        angularOut = std::clamp(angularOut, -maxAng, maxAng);
        lateralOut = std::clamp(lateralOut, -maxLat, maxLat);
        if (!close) {
            lateralOut = std::clamp(lateralOut, prevLateral - settings.lateralSlew, prevLateral + settings.lateralSlew);
        }
        angularOut = std::clamp(angularOut, prevAngular - settings.angularSlew, prevAngular + settings.angularSlew);

        if (!close && minLat > 0.0 && std::abs(lateralOut) < minLat) {
            const double sign = (params.reversed || lateralError < 0.0) ? -1.0 : 1.0;
            lateralOut = sign * minLat;
        }

        if (close && std::abs(lateralError) < 1.0) {
            break;
        }

        prevLateral = lateralOut;
        prevAngular = angularOut;
        driveScaled(*settings.leftMotors, *settings.rightMotors, lateralOut, angularOut, settings.maxVoltage);
        pros::Task::delay_until(&prevTick, 10);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

} // namespace autumn
