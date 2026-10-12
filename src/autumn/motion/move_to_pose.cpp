#include "autumn/motion/motion_math.hpp"
#include "autumn/motion/move_to_pose.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

namespace {

bool shouldExit(const MotionSettings&) { return motionCancelled() || pros::Task::notify_take(true, 0) != 0; }

Pose carrotPoint(const Pose& target, double lead, double distanceIn) {
    const double th = to_radians(target.theta);
    const double back = lead * distanceIn;
    return Pose(to_inches(target.x) - std::sin(th) * back, to_inches(target.y) - std::cos(th) * back,
                to_degrees(target.theta));
}

} // namespace

void moveToPose(Pose target, Time timeout, MoveToPoseParams params, MotionSettings settings) {
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
    bool close = false;

    while (!shouldExit(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        const double distTarget = to_inches(pose.distance(target));
        if (!close && distTarget < 7.5) {
            close = true;
        }
        if (close && distTarget <= std::max(1.0, to_inches(params.earlyExitRange)) &&
            std::abs(headingErrorDeg(to_degrees(target.theta), to_degrees(pose.theta), TurnDirection::Auto)) < 8.0) {
            break;
        }

        const Pose carrot = close ? target : carrotPoint(target, params.lead, distTarget);
        const double robotDeg = to_degrees(pose.theta) + (params.reversed ? 180.0 : 0.0);
        const double aimDeg = close ? to_degrees(target.theta) : to_degrees(pose.angleTo(carrot));
        const double angularError = headingErrorDeg(aimDeg, robotDeg, TurnDirection::Auto);

        double lateralError = to_inches(pose.distance(carrot));
        const double approachCos =
            std::cos(headingErrorDeg(to_degrees(pose.angleTo(carrot)), to_degrees(pose.theta), TurnDirection::Auto) *
                     M_PI / 180.0);
        if (close) {
            lateralError *= approachCos;
        } else {
            lateralError *= approachCos >= 0.0 ? 1.0 : -1.0;
        }

        double lateralOut = std::clamp(lateral.calculate(lateralError), -maxLat, maxLat);
        double angularOut = std::clamp(angular.calculate(angularError), -maxAng, maxAng);
        if (!close) {
            lateralOut = std::clamp(lateralOut, prevLateral - settings.lateralSlew, prevLateral + settings.lateralSlew);
            const double curvature = lookaheadCurvature(pose, carrot);
            const double radius = 1.0 / std::max(std::abs(curvature), 1e-4);
            // LemLib compares this to a 0-127 motor command. Scale into maxVoltage units.
            const double slip = std::sqrt(std::max(0.0, params.horizontalDrift) * radius * 9.8);
            const double slipCap = std::min(maxLat, slip / 127.0 * maxLat);
            lateralOut = std::clamp(lateralOut, -slipCap, slipCap);
            const double overturn = std::abs(angularOut) + std::abs(lateralOut) - maxLat;
            if (overturn > 0.0) {
                lateralOut -= lateralOut > 0.0 ? overturn : -overturn;
            }
        }
        if (!params.reversed && !close) {
            lateralOut = std::max(lateralOut, 0.0);
        } else if (params.reversed && !close) {
            lateralOut = std::min(lateralOut, 0.0);
        }
        if (!params.reversed && lateralOut > 0.0 && lateralOut < minLat) {
            lateralOut = minLat;
        }

        prevLateral = lateralOut;
        driveScaled(*settings.leftMotors, *settings.rightMotors, lateralOut, angularOut, settings.maxVoltage);
        pros::Task::delay_until(&prevTick, 10);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

} // namespace autumn
