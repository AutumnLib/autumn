#include "autumn/motion/follow.hpp"
#include "autumn/motion/motion_math.hpp"

#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <string>

namespace autumn {

namespace {

bool shouldExit(const MotionSettings&) { return motionCancelled() || pros::Task::notify_take(true, 0) != 0; }

double circleIntersect(const Pose& p1, const Pose& p2, const Pose& pose, double lookahead) {
    const double dx = to_inches(p2.x) - to_inches(p1.x);
    const double dy = to_inches(p2.y) - to_inches(p1.y);
    const double fx = to_inches(p1.x) - to_inches(pose.x);
    const double fy = to_inches(p1.y) - to_inches(pose.y);
    const double a = dx * dx + dy * dy;
    const double b = 2.0 * (fx * dx + fy * dy);
    const double c = fx * fx + fy * fy - lookahead * lookahead;
    if (a < 1e-9) {
        return -1.0;
    }
    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
        return -1.0;
    }
    discriminant = std::sqrt(discriminant);
    const double t2 = (-b + discriminant) / (2.0 * a);
    const double t1 = (-b - discriminant) / (2.0 * a);
    if (t2 >= 0.0 && t2 <= 1.0) {
        return t2;
    }
    if (t1 >= 0.0 && t1 <= 1.0) {
        return t1;
    }
    return -1.0;
}

Pose lerpPoint(const Pose& a, const Pose& b, double t) {
    return Pose(to_inches(a.x) + (to_inches(b.x) - to_inches(a.x)) * t,
                to_inches(a.y) + (to_inches(b.y) - to_inches(a.y)) * t);
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
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (line.empty() || line[0] == '#') {
            continue;
        }
        if (line == "endData") {
            break;
        }
        double x = 0.0;
        double y = 0.0;
        double speed = 0.0;
        if (std::sscanf(line.c_str(), "%lf , %lf , %lf", &x, &y, &speed) >= 2 ||
            std::sscanf(line.c_str(), "%lf,%lf,%lf", &x, &y, &speed) >= 2) {
            points.push_back(PathPoint{inches(x), inches(y), ips(speed)});
        }
    }
    return points;
}

void follow(const asset& pathAsset, Length lookahead, Time timeout, bool reversed, FollowParams params,
            MotionSettings settings) {
    const std::vector<PathPoint> path = parsePathAsset(pathAsset);
    if (path.size() < 2 || settings.leftMotors == nullptr || settings.rightMotors == nullptr ||
        settings.poseGetter == nullptr) {
        return;
    }

    const double lookaheadIn = std::max(to_inches(lookahead), 1.0);
    const double maxScale = settings.maxVoltage * std::max(0.05, params.maxLateralSpeed.value());
    const std::uint32_t start = pros::millis();
    std::uint32_t prevTick = start;
    std::size_t lastIndex = 0;
    Pose lastLookahead(path.front().x, path.front().y);

    while (!shouldExit(settings)) {
        if (pros::millis() - start > static_cast<std::uint32_t>(to_milliseconds(timeout))) {
            break;
        }

        Pose pose = settings.poseGetter();
        if (reversed) {
            pose.theta = pose.theta + 180_deg;
        }

        std::size_t closest = 0;
        Length closestDist = pose.distance(Pose(path[0].x, path[0].y));
        for (std::size_t i = 1; i < path.size(); ++i) {
            const Length dist = pose.distance(Pose(path[i].x, path[i].y));
            if (dist < closestDist) {
                closestDist = dist;
                closest = i;
            }
        }
        if (path[closest].speed.value() == 0.0 && closest + 1 == path.size() &&
            to_inches(closestDist) <= std::max(to_inches(params.earlyExitRange), 0.5)) {
            break;
        }
        if (pose.distance(Pose(path.back().x, path.back().y)) <= params.earlyExitRange) {
            break;
        }

        const std::size_t startIndex = std::max(closest, lastIndex);
        Pose lookaheadPose = lastLookahead;
        bool found = false;
        for (std::size_t i = startIndex; i + 1 < path.size(); ++i) {
            const Pose a(path[i].x, path[i].y);
            const Pose b(path[i + 1].x, path[i + 1].y);
            const double t = circleIntersect(a, b, pose, lookaheadIn);
            if (t >= 0.0) {
                lookaheadPose = lerpPoint(a, b, t);
                lastIndex = i;
                found = true;
                break;
            }
        }
        if (!found) {
            lookaheadPose = Pose(path.back().x, path.back().y);
        }
        lastLookahead = lookaheadPose;

        const double curvature = lookaheadCurvature(pose, lookaheadPose);
        double speed = path[closest].speed.value();
        if (std::abs(speed) < 1.0) {
            for (std::size_t i = closest; i-- > 0;) {
                if (std::abs(path[i].speed.value()) >= 1.0) {
                    speed = path[i].speed.value();
                    break;
                }
            }
        }
        // Path files store 0-127, the same scale LemLib uses.
        const double power = std::clamp(speed / 127.0, -1.0, 1.0) * maxScale;
        const double track = std::max(to_inches(settings.trackWidth), 1e-3);
        double left = power * (2.0 + curvature * track) / 2.0;
        double right = power * (2.0 - curvature * track) / 2.0;
        const double peak = std::max(std::abs(left), std::abs(right));
        if (peak > maxScale && peak > 1e-6) {
            left *= maxScale / peak;
            right *= maxScale / peak;
        }
        if (reversed) {
            driveScaled(*settings.leftMotors, *settings.rightMotors, -(left + right) / 2.0, -(left - right) / 2.0,
                        settings.maxVoltage);
        } else {
            driveScaled(*settings.leftMotors, *settings.rightMotors, (left + right) / 2.0, (left - right) / 2.0,
                        settings.maxVoltage);
        }
        pros::Task::delay_until(&prevTick, 10);
    }

    tankDrive(*settings.leftMotors, *settings.rightMotors, 0, 0);
}

} // namespace autumn
