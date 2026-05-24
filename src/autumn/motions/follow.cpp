#include "autumn/control/differential_drive_kinematics.hpp"
#include "autumn/control/ltv_unicycle_controller.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/control/velocity_drive.hpp"
#include "autumn/motion/follow.hpp"
#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/profile/trapezoid_profile.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

namespace autumn {

namespace {

bool shouldExitMotion(const MotionSettings& settings) {
    return motionCancelled() || pros::Task::notify_take(true, 0) != 0;
}

Pose lookaheadPoint(const std::vector<PathPoint>& path, const Pose& robot, Length lookahead) {
    if (path.empty()) {
        return robot;
    }

    std::size_t closest = 0;
    Length closestDist = robot.distance(Pose(path[0].x, path[0].y));
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Length dist = robot.distance(Pose(path[i].x, path[i].y));
        if (dist < closestDist) {
            closestDist = dist;
            closest = i;
        }
    }

    for (std::size_t i = closest; i < path.size(); ++i) {
        const Pose point(path[i].x, path[i].y);
        if (robot.distance(point) >= lookahead) {
            return point;
        }
    }

    return Pose(path.back().x, path.back().y);
}

Pose boomerangTarget(const Pose& pose, const Pose& end, double leadInches) {
    const double dist = to_inches(pose.distance(end));
    const double ratio = (dist > 1e-6) ? std::min(1.0, leadInches / dist) : 1.0;
    const double x = to_inches(pose.x) + ratio * (to_inches(end.x) - to_inches(pose.x));
    const double y = to_inches(pose.y) + ratio * (to_inches(end.y) - to_inches(pose.y));
    return Pose(x, y, to_degrees(pose.angleTo(end)));
}

void followPurePursuitPid(const std::vector<PathPoint>& path, Length lookahead, Time timeout, bool reversed,
                          FollowParams params, MotionSettings settings) {
    PID angular = settings.angularPid != nullptr ? *settings.angularPid : PID(3, 0, 20);
    PID lateral = settings.lateralPid != nullptr ? *settings.lateralPid : PID(10, 0, 3);
    angular.reset();
    lateral.reset();

    const std::uint32_t start = pros::millis();
    const Pose end(path.back().x, path.back().y);

    while (!shouldExitMotion(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        if (pose.distance(end) <= params.earlyExitRange) {
            break;
        }

        Pose target;
        if (params.mode == FollowMode::Boomerang) {
            target = boomerangTarget(pose, end, to_inches(lookahead));
        } else {
            target = lookaheadPoint(path, pose, lookahead);
        }

        const double direction = reversed ? -1.0 : 1.0;
        const double forward = std::clamp(lateral.update(to_inches(pose.distance(target)) * direction),
                                          -settings.maxVoltage * params.maxLateralSpeed.value(),
                                          settings.maxVoltage * params.maxLateralSpeed.value());
        const double turn = std::clamp(angular.update(to_degrees(shortestAngleError(pose.angleTo(target), pose.theta))),
                                       -settings.maxVoltage * params.maxAngularSpeed.value(),
                                       settings.maxVoltage * params.maxAngularSpeed.value());
        arcadeDrive(*settings.leftMotors, *settings.rightMotors, forward, turn, settings.maxVoltage);
        pros::delay(10);
    }
}

void followLtvPath(const std::vector<PathPoint>& path, Length lookahead, Time timeout, bool reversed,
                   FollowParams params, MotionSettings settings) {
    if (settings.leftVelocityPid == nullptr || settings.rightVelocityPid == nullptr ||
        settings.driveFeedforward == nullptr) {
        followPurePursuitPid(path, lookahead, timeout, reversed, params, settings);
        return;
    }

    LtvUnicycleController ltv(params.ltvMaxXErrorIn, params.ltvMaxYErrorIn, params.ltvMaxThetaErrorRad);
    DifferentialDriveKinematics kinematics(to_inches(settings.trackWidth));
    TrapezoidProfile profile(ips(48), 80.0);

    PID leftVel = *settings.leftVelocityPid;
    PID rightVel = *settings.rightVelocityPid;
    Feedforward ff = *settings.driveFeedforward;
    leftVel.reset();
    rightVel.reset();

    const std::uint32_t start = pros::millis();
    const Pose end(path.back().x, path.back().y);
    std::size_t segmentIndex = 0;

    while (!shouldExitMotion(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        const Pose pose = settings.poseGetter();
        if (pose.distance(end) <= params.earlyExitRange) {
            break;
        }

        Pose refPose;
        if (params.mode == FollowMode::Boomerang) {
            refPose = boomerangTarget(pose, end, to_inches(lookahead));
        } else {
            refPose = lookaheadPoint(path, pose, lookahead);
        }

        while (segmentIndex < path.size() - 1 &&
               pose.distance(Pose(path[segmentIndex + 1].x, path[segmentIndex + 1].y)) < 3_in) {
            ++segmentIndex;
        }

        const PathPoint& segEnd = path[std::min(segmentIndex + 1, path.size() - 1)];
        double vRef = segEnd.speed.value();
        if (vRef < 1e-3) {
            vRef = 24.0;
        }
        if (reversed) {
            vRef = -vRef;
        }

        const double dx = to_inches(segEnd.x) - to_inches(refPose.x);
        const double dy = to_inches(segEnd.y) - to_inches(refPose.y);
        const double headingRef = std::atan2(dy, dx);
        const double track = to_inches(settings.trackWidth);
        const double omegaRef = (track > 1e-6) ? vRef / track : 0.0;

        profile.recalculate(pose.distance(refPose));
        const Time elapsed = milliseconds(pros::millis() - start);
        const Velocity profileVel = profile.velocityAt(elapsed);
        if (profileVel.value() > 1e-3) {
            vRef = (vRef > 0.0 ? 1.0 : -1.0) * profileVel.value();
        }

        const ChassisSpeeds speeds = ltv.calculate(pose, refPose.withTheta(radians(headingRef)), vRef, omegaRef);
        const WheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        driveVelocity(*settings.leftMotors, *settings.rightMotors, wheelSpeeds, leftVel, rightVel, ff,
                      settings.maxVoltage);
        pros::delay(10);
    }
}

} // namespace

std::vector<PathPoint> parsePathAsset(const asset& pathAsset) {
    std::vector<PathPoint> points;
    if (pathAsset.buf == nullptr || pathAsset.size == 0) {
        return points;
    }

    std::string content(reinterpret_cast<const char*>(pathAsset.buf), pathAsset.size);
    std::istringstream stream(content);
    std::string line;
    while (std::getline(stream, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        double x = 0.0;
        double y = 0.0;
        double speed = 0.0;
        if (std::sscanf(line.c_str(), "%lf,%lf,%lf", &x, &y, &speed) >= 2) {
            points.push_back(PathPoint{inches(x), inches(y), ips(speed)});
        }
    }
    return points;
}

void follow(const asset& pathAsset, Length lookahead, Time timeout, bool reversed, FollowParams params,
            MotionSettings settings) {
    const std::vector<PathPoint> path = parsePathAsset(pathAsset);
    if (path.empty() || settings.leftMotors == nullptr || settings.rightMotors == nullptr ||
        settings.poseGetter == nullptr) {
        return;
    }

    if (params.useLtvTracking) {
        followLtvPath(path, lookahead, timeout, reversed, params, settings);
    } else {
        followPurePursuitPid(path, lookahead, timeout, reversed, params, settings);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

} // namespace autumn
