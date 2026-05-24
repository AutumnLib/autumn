#include "autumn/motion/profile/trapezoid_profile.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

TrapezoidProfile::TrapezoidProfile(Velocity maxVelocity, double maxAcceleration)
    : maxVelocity_(maxVelocity), maxAcceleration_(maxAcceleration) {}

void TrapezoidProfile::setMaxVelocity(Velocity maxVelocity) { maxVelocity_ = maxVelocity; }

void TrapezoidProfile::setMaxAcceleration(double maxAcceleration) { maxAcceleration_ = maxAcceleration; }

void TrapezoidProfile::recalculate(Length distance) {
    distance_ = distance;
    const double d = to_inches(distance_);
    const double vMax = maxVelocity_.value();
    const double a = std::max(maxAcceleration_, 1e-6);

    const double accelDistance = (vMax * vMax) / a;
    if (d <= accelDistance) {
        accelTime_ = milliseconds(std::sqrt(d / a) * 1000.0);
        cruiseTime_ = 0_ms;
    } else {
        accelTime_ = milliseconds((vMax / a) * 1000.0);
        const double cruiseDistance = d - accelDistance;
        cruiseTime_ = milliseconds((cruiseDistance / vMax) * 1000.0);
    }
    totalTime_ = milliseconds(to_milliseconds(accelTime_) + to_milliseconds(cruiseTime_) + to_milliseconds(accelTime_));
}

Velocity TrapezoidProfile::velocityAt(Time elapsed) const {
    const double t = to_seconds(elapsed);
    const double tAccel = to_seconds(accelTime_);
    const double vMax = maxVelocity_.value();
    const double a = maxAcceleration_;

    if (t <= tAccel) {
        return ips(a * t);
    }
    if (t <= tAccel + to_seconds(cruiseTime_)) {
        return maxVelocity_;
    }
    const double tDecel = t - tAccel - to_seconds(cruiseTime_);
    return ips(std::max(0.0, vMax - a * tDecel));
}

Length TrapezoidProfile::distanceAt(Time elapsed) const {
    const double t = to_seconds(elapsed);
    const double tAccel = to_seconds(accelTime_);
    const double vMax = maxVelocity_.value();
    const double a = maxAcceleration_;

    if (t <= tAccel) {
        return inches(0.5 * a * t * t);
    }
    const double accelDistance = 0.5 * a * tAccel * tAccel;
    if (t <= tAccel + to_seconds(cruiseTime_)) {
        return inches(accelDistance + vMax * (t - tAccel));
    }
    const double tDecel = t - tAccel - to_seconds(cruiseTime_);
    const double cruiseDistance = vMax * to_seconds(cruiseTime_);
    return inches(accelDistance + cruiseDistance + vMax * tDecel - 0.5 * a * tDecel * tDecel);
}

SCurveProfile::SCurveProfile(Velocity maxVelocity, double maxJerk)
    : maxVelocity_(maxVelocity), maxJerk_(maxJerk) {}

Velocity SCurveProfile::velocityAt(Time elapsed) const {
    const double t = to_seconds(elapsed);
    const double tTotal = to_seconds(totalTime_);
    if (t >= tTotal) {
        return ips(0);
    }
    const double normalized = t / tTotal;
    const double smooth = 3.0 * normalized * normalized - 2.0 * normalized * normalized * normalized;
    return ips(maxVelocity_.value() * smooth);
}

} // namespace autumn
