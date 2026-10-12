#pragma once

#include "autumn/chassis/drive_curve.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/localization/odometry.hpp"
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
          lateralPid_(lateral.kP, lateral.kI, lateral.kD, lateral.windupRange, true),
          angularPid_(angular.kP, angular.kI, angular.kD, angular.windupRange, true),
          sensors_(sensors) {
        (void)sensors_.vertical2;
        (void)sensors_.horizontal2;
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
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

    void moveToPose(Pose target, Time timeout, MoveToPoseParams params = {}, bool async = true) {
        params.horizontalDrift = drivetrain_.horizontalDrift;
        auto run = [=, this] { autumn::moveToPose(target, timeout, params, settings()); };
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

    void turnToHeading(Angle target, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::turnToHeading(target, timeout, params, settings()); };
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

    void turnToPoint(Pose target, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::turnToPoint(target, timeout, params, settings()); };
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

    void swingToHeading(Angle target, SwingSide side, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::swingToHeading(target, side, timeout, params, settings()); };
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

    void swingToPoint(Pose target, SwingSide side, Time timeout, TurnToParams params = {}, bool async = true) {
        auto run = [=, this] { autumn::swingToPoint(target, side, timeout, params, settings()); };
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

    void follow(const asset& path, Length lookahead, Time timeout, bool reversed = false, FollowParams params = {},
                bool async = true) {
        auto run = [=, this] { autumn::follow(path, lookahead, timeout, reversed, params, settings()); };
        if (async) {
            motion_handler::move(run);
        } else {
            run();
        }
    }

private:
    MotionSettings settings() {
        MotionSettings out;
        out.poseGetter = [this] { return getPose(); };
        out.leftMotors = drivetrain_.left;
        out.rightMotors = drivetrain_.right;
        out.trackWidth = inches(drivetrain_.trackWidthIn);
        out.lateralPid = &lateralPid_;
        out.angularPid = &angularPid_;
        out.maxVoltage = 127.0;
        out.lateralSlew = 1.0e9;
        out.angularSlew = 1.0e9;
        return out;
    }

    LegacyDrivetrain drivetrain_;
    Odometry& odometry_;
    ExpoDriveCurve throttle_;
    ExpoDriveCurve steer_;
    PID lateralPid_;
    PID angularPid_;
    OdomSensorSet sensors_;
};

} // namespace autumn
