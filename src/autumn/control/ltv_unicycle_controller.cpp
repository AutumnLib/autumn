#include "autumn/control/ltv_unicycle_controller.hpp"

#include "autumn/util/units.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

LtvUnicycleController::LtvUnicycleController(double maxXErrorIn, double maxYErrorIn, double maxThetaErrorRad,
                                           double maxVelocityIps, double maxAngularVelocityRadps)
    : maxXErrorIn_(maxXErrorIn),
      maxYErrorIn_(maxYErrorIn),
      maxThetaErrorRad_(maxThetaErrorRad),
      maxVelocityIps_(maxVelocityIps),
      maxAngularVelocityRadps_(maxAngularVelocityRadps) {}

ChassisSpeeds LtvUnicycleController::calculate(const Pose& currentPose, const Pose& referencePose,
                                              double linearVelocityRefIps,
                                              double angularVelocityRefRadps) const {
    const double dx = to_inches(referencePose.x) - to_inches(currentPose.x);
    const double dy = to_inches(referencePose.y) - to_inches(currentPose.y);
    const double theta = to_radians(currentPose.theta);

    const double cosT = std::cos(theta);
    const double sinT = std::sin(theta);
    const double ex = cosT * dx + sinT * dy;
    const double ey = -sinT * dx + cosT * dy;
    const double eTheta = to_radians(shortestAngleError(referencePose.theta, currentPose.theta));

    const double kx = 1.0 / (maxXErrorIn_ * maxXErrorIn_);
    const double ky = 1.0 / (maxYErrorIn_ * maxYErrorIn_);
    const double kTheta = 1.0 / (maxThetaErrorRad_ * maxThetaErrorRad_);

    ChassisSpeeds speeds;
    speeds.vx = linearVelocityRefIps * std::cos(eTheta) + kx * ex;
    speeds.omega = angularVelocityRefRadps + kTheta * eTheta + ky * linearVelocityRefIps * ey;
    speeds.vy = 0.0;

    speeds.vx = std::clamp(speeds.vx, -maxVelocityIps_, maxVelocityIps_);
    speeds.omega = std::clamp(speeds.omega, -maxAngularVelocityRadps_, maxAngularVelocityRadps_);
    return speeds;
}

} // namespace autumn
