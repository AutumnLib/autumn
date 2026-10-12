#pragma once

#include "autumn/control/chassis_speeds.hpp"
#include "autumn/util/pose.hpp"

namespace autumn {

/// Deferred. The intended controller is a WPILib-style discrete LTV unicycle:
/// linearize the unicycle error dynamics about the reference (v, omega), solve
/// the discrete algebraic Riccati equation over a grid of speeds, and apply the
/// resulting state feedback. Output is chassis velocity (in/s, rad/s), not volts.
/// The previous body was a Kanayama tracker with 1/maxError^2 gains and was not LTV.
/// follow() does not call this. A later series can fill calculate() in.
class LtvUnicycleController {
public:
    LtvUnicycleController(double maxXErrorIn = 0.0625 * 39.3701, double maxYErrorIn = 0.125 * 39.3701,
                          double maxThetaErrorRad = 2.0, double maxVelocityIps = 39.3701,
                          double maxAngularVelocityRadps = 2.0)
        : maxXErrorIn_(maxXErrorIn),
          maxYErrorIn_(maxYErrorIn),
          maxThetaErrorRad_(maxThetaErrorRad),
          maxVelocityIps_(maxVelocityIps),
          maxAngularVelocityRadps_(maxAngularVelocityRadps) {}

    ChassisSpeeds calculate(const Pose&, const Pose&, double, double) const { return {}; }

private:
    double maxXErrorIn_;
    double maxYErrorIn_;
    double maxThetaErrorRad_;
    double maxVelocityIps_;
    double maxAngularVelocityRadps_;
};

} // namespace autumn
