#pragma once

#include "autumn/control/feedforward.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/control/chassis_speeds.hpp"
#include "autumn/control/differential_drive_kinematics.hpp"

#include "pros/motor_group.hpp"

namespace autumn {

void driveVelocity(pros::MotorGroup& left, pros::MotorGroup& right, const WheelSpeeds& targetIps,
                   PID& leftVelocityPid, PID& rightVelocityPid, Feedforward& feedforward, double maxVoltage = 127.0);

} // namespace autumn
