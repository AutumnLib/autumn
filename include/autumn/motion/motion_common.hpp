#pragma once

#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include "pros/motor_group.hpp"

#include <functional>

namespace autumn {

class PID;
class Feedforward;

struct MotionSettings {
    std::function<Pose()> poseGetter;
    pros::MotorGroup* leftMotors = nullptr;
    pros::MotorGroup* rightMotors = nullptr;
    Length trackWidth = 10.5_in;
    PID* lateralPid = nullptr;
    PID* angularPid = nullptr;
    PID* leftVelocityPid = nullptr;
    PID* rightVelocityPid = nullptr;
    Feedforward* driveFeedforward = nullptr;
    double maxVoltage = 127.0;
    double lateralSlew = 5.0;
    double angularSlew = 5.0;
};

void tankDrive(pros::MotorGroup& left, pros::MotorGroup& right, double leftVoltage, double rightVoltage);

void arcadeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double forward, double turn, double maxVoltage = 127.0);

bool motionCancelled();
void setMotionCancelled(bool cancelled);

} // namespace autumn
