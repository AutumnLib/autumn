#pragma once

#include "autumn/chassis/drive_curve.hpp"
#include "autumn/control/feedforward.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/core/context.hpp"
#include "autumn/localization/odometry.hpp"
#include "autumn/motion/follow.hpp"
#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/motion_handler.hpp"
#include "autumn/motion/move_to_point.hpp"
#include "autumn/motion/move_to_pose.hpp"
#include "autumn/motion/turn_to.hpp"
#include "autumn/util/asset.hpp"

#include "pros/motor_group.hpp"

#include <cstdint>
#include <functional>

namespace autumn {

template <typename Config>
class Chassis {
public:
    Chassis(RobotContext<Config>& context, Odometry& odometry, pros::MotorGroup& left, pros::MotorGroup& right,
            ExpoDriveCurve throttleCurve = ExpoDriveCurve(), ExpoDriveCurve steerCurve = ExpoDriveCurve())
        : context_(context),
          odometry_(odometry),
          left_(left),
          right_(right),
          throttleCurve_(throttleCurve),
          steerCurve_(steerCurve),
          lateralPid_(context.config().pidDefaults.lateral),
          angularPid_(context.config().pidDefaults.angular),
          leftVelocityPid_(2.0, 0.0, 0.0),
          rightVelocityPid_(2.0, 0.0, 0.0),
          driveFeedforward_(0.5, 0.0, 2.0) {}

    void calibrate() { odometry_.calibrateImu(); }

    Pose getPose() const { return odometry_.getPose(); }
    void setPose(const Pose& pose) { odometry_.setPose(pose); }
    void setPose(double xInches, double yInches, double headingDegrees = 0.0) {
        setPose(Pose(xInches, yInches, headingDegrees));
    }

    void arcade(double forward, double turn) {
        arcadeDrive(left_, right_, throttleCurve_.curve(forward), steerCurve_.curve(turn));
    }

    void curvature(double forward, double turn) {
        const double f = throttleCurve_.curve(forward);
        const double t = steerCurve_.curve(turn);
        if (std::abs(t) < 1e-3) {
            tankDrive(left_, right_, f, f);
            return;
        }
        const double track = context_.drivetrain().trackWidthIn;
        const double ratio = (2.0 + std::abs(t) / 127.0) / (2.0 - std::abs(t) / 127.0);
        const double leftOut = f * ratio;
        const double rightOut = f;
        tankDrive(left_, right_, leftOut, rightOut);
    }

    MotionSettings motionSettings() {
        return MotionSettings{
            .poseGetter = [this] { return getPose(); },
            .leftMotors = &left_,
            .rightMotors = &right_,
            .trackWidth = context_.drivetrain().trackWidth(),
            .lateralPid = &lateralPid_,
            .angularPid = &angularPid_,
            .leftVelocityPid = &leftVelocityPid_,
            .rightVelocityPid = &rightVelocityPid_,
            .driveFeedforward = &driveFeedforward_,
        };
    }

    void moveToPoint(Pose target, Time timeout, MoveToPointParams params = {}) {
        motion_handler::move([=, this] { autumn::moveToPoint(target, timeout, params, motionSettings()); });
    }

    void moveToPoint(double xInches, double yInches, std::uint32_t timeoutMs, MoveToPointParams params = {}) {
        moveToPoint(Pose(xInches, yInches), milliseconds(timeoutMs), params);
    }

    void moveToPose(Pose target, Time timeout, MoveToPoseParams params = {}) {
        motion_handler::move([=, this] { autumn::moveToPose(target, timeout, params, motionSettings()); });
    }

    void moveToPose(double xInches, double yInches, double headingDegrees, std::uint32_t timeoutMs,
                    MoveToPoseParams params = {}) {
        moveToPose(Pose(xInches, yInches, headingDegrees), milliseconds(timeoutMs), params);
    }

    void turnToHeading(Angle target, Time timeout, TurnToParams params = {}) {
        motion_handler::move([=, this] { autumn::turnToHeading(target, timeout, params, motionSettings()); });
    }

    void turnToHeading(double headingDegrees, std::uint32_t timeoutMs, TurnToParams params = {}) {
        turnToHeading(degrees(headingDegrees), milliseconds(timeoutMs), params);
    }

    void turnToPoint(Pose target, Time timeout, TurnToParams params = {}) {
        motion_handler::move([=, this] { autumn::turnToPoint(target, timeout, params, motionSettings()); });
    }

    void turnToPoint(double xInches, double yInches, std::uint32_t timeoutMs, TurnToParams params = {}) {
        turnToPoint(Pose(xInches, yInches), milliseconds(timeoutMs), params);
    }

    void follow(const asset& pathAsset, Length lookahead, Time timeout, bool reversed = false,
                FollowParams params = {}) {
        motion_handler::move(
            [=, this] { autumn::follow(pathAsset, lookahead, timeout, reversed, params, motionSettings()); });
    }

    void follow(const asset& pathAsset, double lookaheadInches, std::uint32_t timeoutMs, bool reversed = false,
                FollowParams params = {}) {
        follow(pathAsset, inches(lookaheadInches), milliseconds(timeoutMs), reversed, params);
    }

    void waitUntilDone() {
        while (motion_handler::isMoving()) {
            pros::delay(5);
        }
    }

    void waitUntilPoint(Pose target, Length radius) {
        motion_handler::waitUntilPoint(target, radius, [this] { return getPose(); });
    }

    void waitUntilPoint(double xInches, double yInches, double radiusInches) {
        waitUntilPoint(Pose(xInches, yInches), inches(radiusInches));
    }

    void waitUntilDistance(Length distance) {
        motion_handler::waitUntilDistance(distance, [this] { return getPose(); });
    }

    void waitUntilDistance(double distanceInches) { waitUntilDistance(inches(distanceInches)); }

    void cancel() { motion_handler::cancel(); }

private:
    RobotContext<Config>& context_;
    Odometry& odometry_;
    pros::MotorGroup& left_;
    pros::MotorGroup& right_;
    ExpoDriveCurve throttleCurve_;
    ExpoDriveCurve steerCurve_;
    PID lateralPid_;
    PID angularPid_;
    PID leftVelocityPid_;
    PID rightVelocityPid_;
    Feedforward driveFeedforward_;
};

} // namespace autumn
