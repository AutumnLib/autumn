#pragma once

#include "autumn/control/chassis_speeds.hpp"
#include "autumn/util/pose.hpp"

namespace autumn {

/// WPILib-style LTV unicycle tracker (see WPILib LTVUnicycleController).
/// Returns chassis velocities (in/s, rad/s), not motor voltages.
class LtvUnicycleController {
public:
  LtvUnicycleController(double maxXErrorIn = 0.0625 * 39.3701, double maxYErrorIn = 0.125 * 39.3701,
                      double maxThetaErrorRad = 2.0, double maxVelocityIps = 39.3701,
                      double maxAngularVelocityRadps = 2.0);

  ChassisSpeeds calculate(const Pose& currentPose, const Pose& referencePose, double linearVelocityRefIps,
                          double angularVelocityRefRadps) const;

private:
  double maxXErrorIn_;
  double maxYErrorIn_;
  double maxThetaErrorRad_;
  double maxVelocityIps_;
  double maxAngularVelocityRadps_;
};

} // namespace autumn
