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
    // Full-scale command in millivolts, matching the PID output unit. Motions clamp
    // to the param fractions of this value and hand millivolts to the motor.
    double maxVoltage = 12000.0;
    // Command change allowed per 10 ms tick, in millivolts. Defaults match the
    // LemLib 0.5 rate of 5 per tick on the 0-127 scale (5/127 of full scale).
    double lateralSlew = 12000.0 * 5.0 / 127.0;
    double angularSlew = 12000.0 * 5.0 / 127.0;
    // Dual exit bands (LemLib ControllerSettings). small* are the tighter bands.
    Length lateralSmallSettleError = 1_in;
    Time lateralSmallSettleTime = 500_ms;
    Length lateralLargeSettleError = 3_in;
    Time lateralLargeSettleTime = 100_ms;
    Angle angularSmallSettleError = 8_deg;
    Time angularSmallSettleTime = 500_ms;
    Angle angularLargeSettleError = 3_deg;
    Time angularLargeSettleTime = 100_ms;
    // Legacy field still accepted by facades; maps to small bands when non-default.
    Length lateralSettleError = 1_in;
    Angle angularSettleError = 8_deg;
    // Stall exit: 0_ms disables (RW-Template stall_exit_msec pattern).
    Time stallExitTime = 0_ms;
    double stallGraceMs = 300.0;
    Length stallSpeedThreshold = 1.5_in;
    Angle stallTurnThreshold = 10_deg;
    // LemLib 0.5 controller gains are tuned against the 0-127 motor command scale,
    // but PID output here is compared against millivolt limits. Scaling the raw
    // controller output by this factor (12000 / 127 ~ 94.5) lets LemLib-style
    // gains drive the same fraction of full power. Set to 1 to work in millivolts
    // directly.
    double commandScale = 12000.0 / 127.0;
};

void tankDrive(pros::MotorGroup& left, pros::MotorGroup& right, double leftVoltage, double rightVoltage);

void arcadeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double forward, double turn, double maxVoltage = 127.0);

// Mix lateral/angular commands that are in `maxVoltage` units and send millivolts.
void driveScaled(pros::MotorGroup& left, pros::MotorGroup& right, double lateral, double angular, double maxVoltage);

bool motionCancelled();
void setMotionCancelled(bool cancelled);

// Path and heading progress for async wait helpers (LemLib distTraveled / RW waitUntil*).
void resetMotionProgress();
void sampleMotionProgress(const Pose& pose);
double motionPathDistance();
double motionTurnDegrees();

struct StallTracker {
    void arm(std::uint32_t startMs);
    // Returns true when stall_exit should end the motion.
    bool update(const MotionSettings& settings, std::uint32_t nowMs, const Pose& pose, Pose& lastPose);

private:
    std::uint32_t startMs_ = 0;
    std::uint32_t stallSinceMs_ = 0;
};

inline double clampMinLateral(double output, double minVoltage, double errorSign = 0.0) {
    if (minVoltage <= 0.0) {
        return output;
    }
    if (output > 0.0 && output < minVoltage) {
        return minVoltage;
    }
    if (output < 0.0 && output > -minVoltage) {
        return -minVoltage;
    }
    if (std::abs(output) < 1e-6 && std::abs(errorSign) > 1e-6) {
        return errorSign > 0.0 ? minVoltage : -minVoltage;
    }
    return output;
}

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
