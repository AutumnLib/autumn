#pragma once

#include "autumn/control/controller.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include "pros/motor_group.hpp"

#include <functional>

namespace autumn {

class PID;
class Feedforward;

struct MotionSettings {
    std::function<Pose()> poseGetter;
    pros::MotorGroup* leftMotors = nullptr;
    pros::MotorGroup* rightMotors = nullptr;
    Length trackWidth = 10.5_in;
    PID* lateralPid = nullptr;
    PID* angularPid = nullptr;
    PID* leftVelocityPid = nullptr;
    PID* rightVelocityPid = nullptr;
    Feedforward* driveFeedforward = nullptr;
    // Optional composable pipeline. When non-null a motion may route its scalar
    // error through these instead of the raw PID pointers above. Null keeps the
    // default inline behavior for back-compat.
    ScalarController* lateralController = nullptr;
    ScalarController* angularController = nullptr;
    // Full-scale command in the same units as PID output. 1.0 means normalized ±1.
    // Converted to ±12000 mV at the motor. Joystick code still uses ±127 and scales once.
    double maxVoltage = 1.0;
    double lateralSlew = 5.0;
    double angularSlew = 5.0;
};

void tankDrive(pros::MotorGroup& left, pros::MotorGroup& right, double leftVoltage, double rightVoltage);

void arcadeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double forward, double turn, double maxVoltage = 127.0);

// Mix lateral/angular commands that are in `maxVoltage` units and send millivolts.
void driveScaled(pros::MotorGroup& left, pros::MotorGroup& right, double lateral, double angular, double maxVoltage);

bool motionCancelled();
void setMotionCancelled(bool cancelled);

// Prefer an injected ScalarController; otherwise build a PidController from the
// MotionSettings PID pointers (or the given default gains).
inline PidController makeLateralFallback(const MotionSettings& settings, double kP = 10.0, double kI = 0.0,
                                         double kD = 0.03) {
    return PidController(settings.lateralPid != nullptr ? *settings.lateralPid : PID(kP, kI, kD));
}

inline PidController makeAngularFallback(const MotionSettings& settings, double kP = 3.0, double kI = 0.0,
                                         double kD = 0.2) {
    return PidController(settings.angularPid != nullptr ? *settings.angularPid : PID(kP, kI, kD));
}

inline ScalarController& resolveLateral(MotionSettings& settings, PidController& fallback) {
    return settings.lateralController != nullptr ? *settings.lateralController : fallback;
}

inline ScalarController& resolveAngular(MotionSettings& settings, PidController& fallback) {
    return settings.angularController != nullptr ? *settings.angularController : fallback;
}

} // namespace autumn
