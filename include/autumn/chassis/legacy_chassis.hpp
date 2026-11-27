#pragma once

#include "autumn/chassis/drive_curve.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/localization/odometry.hpp"
#include "autumn/logging/error_codes.hpp"
#include "autumn/motion/follow.hpp"
#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/motion_handler.hpp"
#include "autumn/motion/move_to_point.hpp"
#include "autumn/motion/move_to_pose.hpp"
#include "autumn/motion/turn_to.hpp"

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace autumn {

// Plain-struct chassis. AutumnContext is not required.
struct LegacyDrivetrain {
    pros::MotorGroup* left = nullptr;
    pros::MotorGroup* right = nullptr;
    double trackWidthIn = 10.5;
    double wheelDiameterIn = 4.0;
    double horizontalDrift = 2.0;
};

struct ControllerSettings {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double windupRange = 0.0;
    double smallError = 0.0;
    double smallErrorTimeoutMs = 0.0;
    double largeError = 0.0;
    double largeErrorTimeoutMs = 0.0;
    double slew = 0.0;
};

// v2 and h2 are accepted for the LemLib shape. Odometry still uses one wheel per axis.
struct OdomSensorSet {
    TrackingWheel* vertical1 = nullptr;
    TrackingWheel* vertical2 = nullptr;
    TrackingWheel* horizontal1 = nullptr;
    TrackingWheel* horizontal2 = nullptr;
    pros::Imu* imu = nullptr;
};

class LegacyChassis {
public:
    LegacyChassis(LegacyDrivetrain drivetrain, ControllerSettings lateral, ControllerSettings angular,
                  OdomSensorSet sensors, Odometry& odometry, ExpoDriveCurve throttle = ExpoDriveCurve(),
                  ExpoDriveCurve steer = ExpoDriveCurve())
        : drivetrain_(drivetrain),
          odometry_(odometry),
          throttle_(throttle),
          steer_(steer),
          lateralSettings_(lateral),
          angularSettings_(angular),
          lateralPid_(lateral.kP, lateral.kI, lateral.kD, lateral.windupRange, true),
          angularPid_(angular.kP, angular.kI, angular.kD, angular.windupRange, true),
          sensors_(sensors) {
        // Odometry still reads one wheel per axis, so the second wheel on an axis
        // is unused. Say so rather than silently dropping it.
        if (sensors_.vertical2 != nullptr || sensors_.horizontal2 != nullptr) {
            reportError(ErrorCode::UnsupportedConfig,
                        "LegacyChassis: second tracking wheel on an axis is not used by odometry");
        }
    }

    Pose getPose() const { return odometry_.getPose(); }
    void setPose(const Pose& pose) { odometry_.setPose(pose); }
    void setPose(double xInches, double yInches, double headingDegrees = 0.0) {
        setPose(Pose(xInches, yInches, headingDegrees));
    }

    bool isInMotion() const { return motion_handler::isMoving(); }
    void cancelMotion() { motion_handler::cancel(); }
    void cancelAllMotions() { motion_handler::cancel(); }
    void waitUntilDone() {
        while (motion_handler::isMoving()) {
            pros::delay(5);
        }
    }

    PID& lateralPid() { return lateralPid_; }
    PID& angularPid() { return angularPid_; }

    void moveToPoint(Pose target, Time timeout, MoveToPointParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::moveToPoint(target, timeout, params, settings()); };
        runOrDispatch(run, async);
    }

    void moveToPose(Pose target, Time timeout, MoveToPoseParams params = {}, bool async = true) {
        // LemLib: a zero drift in the params means "use the drivetrain value", so
        // only fill it in when the caller left it unset.
        if (params.horizontalDrift == 0.0) {
            params.horizontalDrift = drivetrain_.horizontalDrift;
        }
        auto run = [=, this] { autumn::moveToPose(target, timeout, params, settings()); };
        runOrDispatch(run, async);
    }

    void turnToHeading(Angle target, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::turnToHeading(target, timeout, params, settings()); };
        runOrDispatch(run, async);
    }

    void turnToPoint(Pose target, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::turnToPoint(target, timeout, params, settings()); };
        runOrDispatch(run, async);
    }

    void swingToHeading(Angle target, SwingSide side, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::swingToHeading(target, side, timeout, params, settings()); };
        runOrDispatch(run, async);
    }

    void swingToPoint(Pose target, SwingSide side, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::swingToPoint(target, side, timeout, params, settings()); };
        runOrDispatch(run, async);
    }

    void follow(const asset& path, Length lookahead, Time timeout, bool reversed = false, FollowParams params = {},
                bool async = true) {
        auto run = [=, this] { autumn::follow(path, lookahead, timeout, reversed, params, settings()); };
        runOrDispatch(run, async);
    }

    // The MotionSettings this facade hands to each motion. Public so a caller can
    // inspect or extend the wiring without duplicating it.
    MotionSettings settings() {
        MotionSettings out;
        out.poseGetter = [this] { return getPose(); };
        out.leftMotors = drivetrain_.left;
        out.rightMotors = drivetrain_.right;
        out.trackWidth = inches(drivetrain_.trackWidthIn);
        out.lateralPid = &lateralPid_;
        out.angularPid = &angularPid_;
        out.maxVoltage = 12000.0;
        out.lateralSlew = slewFor(lateralSettings_);
        out.angularSlew = slewFor(angularSettings_);
        if (lateralSettings_.smallError > 0.0) {
            out.lateralSettleError = inches(lateralSettings_.smallError);
        }
        if (angularSettings_.smallError > 0.0) {
            out.angularSettleError = degrees(angularSettings_.smallError);
        }
        return out;
    }

private:
    // LemLib expresses slew on the 0-127 command scale; motions run in millivolts.
    // A zero slew in LemLib means "unlimited", which is what 1e9 enforces here.
    static double slewFor(const ControllerSettings& controller) {
        if (controller.slew <= 0.0) {
            return 1.0e9;
        }
        return controller.slew / 127.0 * 12000.0;
    }

    void runOrDispatch(std::function<void()> run, bool async) {
        if (async) {
            motion_handler::move(std::move(run));
            return;
        }
        // A blocking motion owns the caller's task, so it must clear any cancel
        // left over from a previous motion and must not depend on the handler's
        // task state. motionCancelled() is still honored inside the motion loop.
        setMotionCancelled(false);
        run();
    }

    LegacyDrivetrain drivetrain_;
    Odometry& odometry_;
    ExpoDriveCurve throttle_;
    ExpoDriveCurve steer_;
    ControllerSettings lateralSettings_;
    ControllerSettings angularSettings_;
    PID lateralPid_;
    PID angularPid_;
    OdomSensorSet sensors_;
};

} // namespace autumn
