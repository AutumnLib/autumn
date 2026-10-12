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
    const double d = std::abs(to_inches(distance_));
    const double vCap = std::max(maxVelocity_.value(), 1e-6);
    const double a = std::max(maxAcceleration_, 1e-6);
    const double fullAccelDistance = (vCap * vCap) / a;

    double vPeak = vCap;
    if (d <= fullAccelDistance) {
        vPeak = std::sqrt(std::max(d, 0.0) * a);
        accelTime_ = milliseconds((vPeak / a) * 1000.0);
        cruiseTime_ = 0_ms;
    } else {
        accelTime_ = milliseconds((vCap / a) * 1000.0);
        cruiseTime_ = milliseconds(((d - fullAccelDistance) / vCap) * 1000.0);
    }
    peakVelocity_ = ips(vPeak);
    totalTime_ = milliseconds(to_milliseconds(accelTime_) + to_milliseconds(cruiseTime_) + to_milliseconds(accelTime_));
}

Velocity TrapezoidProfile::velocityAt(Time elapsed) const {
    const double t = to_seconds(elapsed);
    const double tAccel = to_seconds(accelTime_);
    const double vPeak = peakVelocity_.value();
    const double a = maxAcceleration_;

    if (t <= 0.0 || t >= to_seconds(totalTime_)) {
        return ips(0);
    }
    if (t <= tAccel) {
        return ips(a * t);
    }
    if (t <= tAccel + to_seconds(cruiseTime_)) {
        return peakVelocity_;
    }
    const double tDecel = t - tAccel - to_seconds(cruiseTime_);
    return ips(std::max(0.0, vPeak - a * tDecel));
}

Length TrapezoidProfile::distanceAt(Time elapsed) const {
    const double t = std::min(to_seconds(elapsed), to_seconds(totalTime_));
    const double tAccel = to_seconds(accelTime_);
    const double vPeak = peakVelocity_.value();
    const double a = maxAcceleration_;

    if (t <= tAccel) {
        return inches(0.5 * a * t * t);
    }
    const double accelDistance = 0.5 * a * tAccel * tAccel;
    if (t <= tAccel + to_seconds(cruiseTime_)) {
        return inches(accelDistance + vPeak * (t - tAccel));
    }
    const double tDecel = t - tAccel - to_seconds(cruiseTime_);
    const double cruiseDistance = vPeak * to_seconds(cruiseTime_);
    return inches(accelDistance + cruiseDistance + vPeak * tDecel - 0.5 * a * tDecel * tDecel);
}

SCurveProfile::SCurveProfile(Velocity maxVelocity, double maxAcceleration, double maxJerk)
    : maxVelocity_(maxVelocity), maxAcceleration_(maxAcceleration), maxJerk_(maxJerk) {}

void SCurveProfile::recalculate(Length distance) {
    const double d = std::abs(to_inches(distance));
    const double vCap = std::max(maxVelocity_.value(), 1e-6);
    const double aCap = std::max(maxAcceleration_, 1e-6);
    const double j = std::max(maxJerk_, 1e-6);

    auto shape = [&](double vPeak, double& tJ, double& tA, double& dAcc) {
        tJ = aCap / j;
        tA = vPeak / aCap - tJ;
        if (tA < 0.0) {
            tJ = std::sqrt(std::max(vPeak, 1e-9) / j);
            tA = 0.0;
        }
        const double accel = j * tJ;
        const double v1 = 0.5 * j * tJ * tJ;
        const double d1 = j * tJ * tJ * tJ / 6.0;
        const double d2 = v1 * tA + 0.5 * accel * tA * tA;
        const double v2 = v1 + accel * tA;
        const double d3 = v2 * tJ + 0.5 * accel * tJ * tJ - j * tJ * tJ * tJ / 6.0;
        dAcc = d1 + d2 + d3;
    };

    double lo = 1e-3;
    double hi = vCap;
    double tJ = 0.0;
    double tA = 0.0;
    double dAcc = 0.0;
    for (int i = 0; i < 48; ++i) {
        const double mid = 0.5 * (lo + hi);
        shape(mid, tJ, tA, dAcc);
        if (2.0 * dAcc > d) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    const double vPeak = lo;
    shape(vPeak, tJ, tA, dAcc);
    const double cruiseDistance = std::max(0.0, d - 2.0 * dAcc);
    const double tCruise = cruiseDistance / std::max(vPeak, 1e-6);
    peakVelocity_ = vPeak;
    jerkTime_ = milliseconds(tJ * 1000.0);
    accelTime_ = milliseconds(tA * 1000.0);
    cruiseTime_ = milliseconds(tCruise * 1000.0);
    totalTime_ = milliseconds((4.0 * tJ + 2.0 * tA + tCruise) * 1000.0);
}

Velocity SCurveProfile::velocityAt(Time elapsed) const {
    const double t = to_seconds(elapsed);
    const double tJ = to_seconds(jerkTime_);
    const double tA = to_seconds(accelTime_);
    const double tC = to_seconds(cruiseTime_);
    const double j = std::max(maxJerk_, 1e-6);
    const double a = j * tJ;
    if (t <= 0.0 || t >= to_seconds(totalTime_)) {
        return ips(0);
    }

    auto phaseVelocity = [&](double time) {
        if (time < tJ) {
            return 0.5 * j * time * time;
        }
        if (time < tJ + tA) {
            const double tau = time - tJ;
            return 0.5 * j * tJ * tJ + a * tau;
        }
        if (time < 2.0 * tJ + tA) {
            const double tau = time - tJ - tA;
            const double v2 = 0.5 * j * tJ * tJ + a * tA;
            return v2 + a * tau - 0.5 * j * tau * tau;
        }
        return peakVelocity_;
    };

    const double ramp = 2.0 * tJ + tA;
    if (t < ramp) {
        return ips(phaseVelocity(t));
    }
    if (t < ramp + tC) {
        return ips(peakVelocity_);
    }
    const double down = t - ramp - tC;
    return ips(std::max(0.0, peakVelocity_ - phaseVelocity(down)));
}

Length SCurveProfile::distanceAt(Time) const { return 0_in; }

} // namespace autumn
