#include "autumn/control/velocity_drive.hpp"

#include "autumn/motion/motion_common.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

void driveVelocity(pros::MotorGroup& left, pros::MotorGroup& right, const WheelSpeeds& targetIps,
                   PID& leftVelocityPid, PID& rightVelocityPid, Feedforward& feedforward, double maxVoltage) {
    const double leftMeasured = left.get_actual_velocity();
    const double rightMeasured = right.get_actual_velocity();

    const double leftOut = std::clamp(
        feedforward.calculate(targetIps.left, 0.0) + leftVelocityPid.update(targetIps.left - leftMeasured), -maxVoltage,
        maxVoltage);
    const double rightOut = std::clamp(
        feedforward.calculate(targetIps.right, 0.0) + rightVelocityPid.update(targetIps.right - rightMeasured),
        -maxVoltage, maxVoltage);

    tankDrive(left, right, leftOut, rightOut);
}

} // namespace autumn
