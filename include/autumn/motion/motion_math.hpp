#pragma once

#include "autumn/motion/turn_to.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include <cmath>

namespace autumn {

inline double headingErrorDeg(double targetDeg, double currentDeg, TurnDirection direction) {
    double error = std::remainder(targetDeg - currentDeg, 360.0);
    if (direction == TurnDirection::Clockwise && error < 0.0) {
        error += 360.0;
    } else if (direction == TurnDirection::CounterClockwise && error > 0.0) {
        error -= 360.0;
    }
    return error;
}

inline bool crossedPerpendicular(const Pose& pose, const Pose& target, Angle approach, Length earlyExit) {
    const double dx = to_inches(pose.x) - to_inches(target.x);
    const double dy = to_inches(pose.y) - to_inches(target.y);
    const double th = to_radians(approach);
    return dy * -std::sin(th) <= dx * std::cos(th) + to_inches(earlyExit);
}

inline double lookaheadCurvature(const Pose& pose, const Pose& lookahead) {
    const double heading = M_PI / 2.0 - to_radians(pose.theta);
    const double dx = to_inches(lookahead.x) - to_inches(pose.x);
    const double dy = to_inches(lookahead.y) - to_inches(pose.y);
    const double side = std::sin(heading) * dx - std::cos(heading) * dy;
    const double sign = side >= 0.0 ? 1.0 : -1.0;
    const double a = -std::tan(heading);
    const double c = std::tan(heading) * to_inches(pose.x) - to_inches(pose.y);
    const double x = std::abs(a * to_inches(lookahead.x) + to_inches(lookahead.y) + c) / std::sqrt(a * a + 1.0);
    const double d = std::hypot(dx, dy);
    if (d < 1e-6) {
        return 0.0;
    }
    return sign * ((2.0 * x) / (d * d));
}

} // namespace autumn
