#pragma once

#include "autumn/core/base_config.hpp"
#include "autumn/localization/tracking_wheel.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

struct TeamConfig : autumn::BaseConfig {
    pros::Controller controller{pros::E_CONTROLLER_MASTER};

    pros::MotorGroup leftMotors{-5, 4, -3};
    pros::MotorGroup rightMotors{6, -9, 7};

    pros::Imu imu{10};
    pros::Rotation verticalEnc{-11};
    pros::Rotation horizontalEnc{20};

    autumn::TrackingWheel verticalTracker{&verticalEnc, 2.75, -2.5};
    autumn::TrackingWheel horizontalTracker{&horizontalEnc, 2.75, -5.75};

    autumn::ExpoDriveCurve throttleCurve{3, 10, 1.019};
    autumn::ExpoDriveCurve steerCurve{3, 10, 1.019};
};
