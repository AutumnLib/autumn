#include "autumn/control/exit_condition.hpp"
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

    const Length lateralSmallTol =
        settings.lateralSmallSettleError > 0_in ? settings.lateralSmallSettleError : settings.lateralSettleError;
    const Length lateralLargeTol =
        settings.lateralLargeSettleError > 0_in ? settings.lateralLargeSettleError : inches(3.0);
    const Angle angularSmallTol =
        settings.angularSmallSettleError > 0_deg ? settings.angularSmallSettleError : settings.angularSettleError;
    const Angle angularLargeTol =
        settings.angularLargeSettleError > 0_deg ? settings.angularLargeSettleError : degrees(3.0);

    ExitCondition lateralSmall(to_inches(lateralSmallTol), settings.lateralSmallSettleTime);
    ExitCondition lateralLarge(to_inches(lateralLargeTol), settings.lateralLargeSettleTime);
    ExitCondition angularSmall(to_degrees(angularSmallTol), settings.angularSmallSettleTime);
    ExitCondition angularLarge(to_degrees(angularLargeTol), settings.angularLargeSettleTime);

    double maxLat = settings.maxVoltage * std::max(0.0, params.maxLateralSpeed.value());
    const double maxAng = settings.maxVoltage * std::max(0.0, params.maxAngularSpeed.value());
    const double minLat = settings.maxVoltage * std::abs(params.minLateralSpeed.value());
    const double drift = params.horizontalDrift > 0.0 ? params.horizontalDrift : 2.0;
    const std::uint32_t start = pros::millis();
    std::uint32_t prevTick = start;
    double prevLateral = 0.0;
    bool close = false;
    bool lateralSettled = false;
    bool prevSameSide = false;
    StallTracker stall;
    stall.arm(start);
    Pose lastPose = settings.poseGetter();
    resetMotionProgress();
    sampleMotionProgress(lastPose);

    while (!shouldExit(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        sampleMotionProgress(pose);
        if (stall.update(settings, pros::millis(), pose, lastPose)) {
            break;
        }

        const double distTarget = to_inches(pose.distance(target));
        if (!close && distTarget < 7.5) {
            close = true;
            // LemLib eases lateral power during the settling phase instead of keeping cruise speed.
            maxLat = std::max(std::abs(prevLateral), 60.0 / 127.0 * settings.maxVoltage);
        }
        if (lateralLarge.getExit() && lateralSmall.getExit()) {
            lateralSettled = true;
        }

        const bool distanceSettled = distTarget <= to_inches(lateralSmallTol);
        const bool headingSettled =
            std::abs(headingErrorDeg(to_degrees(target.theta), to_degrees(pose.theta), TurnDirection::Auto)) <
            to_degrees(angularSmallTol);
        if (close && lateralSettled && angularLarge.getExit() && angularSmall.getExit()) {
            break;
        }
        if (close && distanceSettled && headingSettled && minLat <= 0.0) {
            break;
        }

        const Pose carrot = close ? target : carrotPoint(target, params.lead, distTarget);
        const Angle approach = target.theta;
        const bool robotSide = crossedPerpendicular(pose, target, approach, params.earlyExitRange);
        const bool carrotSide = crossedPerpendicular(carrot, target, approach, params.earlyExitRange);
        const bool sameSide = robotSide == carrotSide;
        if (!sameSide && prevSameSide && close && minLat > 0.0) {
            break;
        }
        prevSameSide = sameSide;

        const double robotDeg = to_degrees(pose.theta) + (params.reversed ? 180.0 : 0.0);
        const double toCarrotDeg = to_degrees(pose.angleTo(carrot));

        const bool rotatingInPlace = close && distanceSettled;
        const double aimDeg = rotatingInPlace ? to_degrees(target.theta) : toCarrotDeg;
        const double angularError = headingErrorDeg(aimDeg, robotDeg, TurnDirection::Auto);

        double lateralError = 0.0;
        if (!rotatingInPlace) {
            const double approachCos =
                std::cos(headingErrorDeg(toCarrotDeg, to_degrees(pose.theta), TurnDirection::Auto) * M_PI / 180.0);
            lateralError = to_inches(pose.distance(carrot));
            if (close) {
                lateralError *= std::max(approachCos, 0.0);
            } else {
                lateralError *= approachCos >= 0.0 ? 1.0 : -1.0;
            }
        }

        lateralSmall.update(std::abs(lateralError), 10_ms);
        lateralLarge.update(std::abs(lateralError), 10_ms);
        angularSmall.update(std::abs(angularError), 10_ms);
        angularLarge.update(std::abs(angularError), 10_ms);

        double lateralOut = std::clamp(lateral.calculate(lateralError) * settings.commandScale, -maxLat, maxLat);
        double angularOut = std::clamp(angular.calculate(angularError) * settings.commandScale, -maxAng, maxAng);
        if (!close) {
            lateralOut = std::clamp(lateralOut, prevLateral - settings.lateralSlew, prevLateral + settings.lateralSlew);
            const double curvature = lookaheadCurvature(pose, carrot);
            const double radius = 1.0 / std::max(std::abs(curvature), 1e-4);
            const double slip = std::sqrt(std::max(0.0, drift) * radius * 9.8);
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
        const double lateralSign = lateralError >= 0.0 ? 1.0 : -1.0;
        lateralOut = clampMinLateral(lateralOut, minLat, lateralSign);

        prevLateral = lateralOut;
        driveScaled(*settings.leftMotors, *settings.rightMotors, lateralOut, angularOut, settings.maxVoltage);
        pros::Task::delay_until(&prevTick, 10);
    }

    if (minLat <= 0.0) {
        tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
    }
}

} // namespace autumn