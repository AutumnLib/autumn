#include "autumn/motion/motion_math.hpp"
#include "autumn/motion/turn_to.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

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

    // LemLib holds the locked side still while the other side drives, then puts
    // the original brake mode back. Without this the robot coasts around the
    // swing arc instead of pivoting about the held side.
    pros::MotorGroup* heldMotors = nullptr;
    pros::motor_brake_mode_e_t previousBrakeMode = pros::E_MOTOR_BRAKE_COAST;
    if (locked != nullptr) {
        heldMotors = (*locked == SwingSide::Left) ? settings.leftMotors : settings.rightMotors;
        previousBrakeMode = heldMotors->get_brake_mode();
        heldMotors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    const double maxOut = settings.maxVoltage * std::max(0.0, params.maxSpeed.value());
    const double minOut = settings.maxVoltage * std::abs(params.minSpeed.value());
    const std::uint32_t start = pros::millis();
    std::uint32_t prev = start;
    double prevPower = 0.0;
    // LemLib holds directional error only until the robot first crosses the target.
    // Past that point asking for "clockwise" would add a full turn of error forever,
    // so both the controller and the exit test switch back to the shortest angle.
    bool settling = false;
    std::optional<double> prevShortestError;

    while (!shouldExit(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }
        const Pose pose = settings.poseGetter();
        const double shortestError =
            headingErrorDeg(to_degrees(target), to_degrees(pose.theta), TurnDirection::Auto);
        if (!prevShortestError.has_value()) {
            prevShortestError = shortestError;
        } else if ((shortestError > 0.0) != (*prevShortestError > 0.0)) {
            settling = true;
        }
        prevShortestError = shortestError;

        const double errorDeg = settling ? shortestError
                                         : headingErrorDeg(to_degrees(target), to_degrees(pose.theta),
                                                           params.direction);
        const double earlyExitDeg = to_degrees(params.earlyExitRange);
        if (earlyExitDeg > 0.0 && std::abs(shortestError) <= earlyExitDeg) {
            break;
        }
        if (earlyExitDeg <= 0.0 && std::abs(shortestError) < 0.5) {
            break;
        }

        double power = angular.calculate(errorDeg) * settings.commandScale;
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
            // SwingSide names the side held still, as in LemLib: hold the left and
            // drive the right. Driving both sides (as this used to) is a normal
            // point turn, not a swing.
            tankDrive(*settings.leftMotors, *settings.rightMotors, 0.0, -power);
        } else {
            tankDrive(*settings.leftMotors, *settings.rightMotors, power, 0.0);
        }

        const std::uint32_t period = 10;
        pros::Task::delay_until(&prev, period);
    }

    if (heldMotors != nullptr) {
        heldMotors->set_brake_mode(previousBrakeMode);
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
