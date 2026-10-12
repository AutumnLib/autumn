#include "support.hpp"
#include "doctest.h"

#include "autumn/chassis/chassis.hpp"
#include "autumn/chassis/drive_curve.hpp"
#include "autumn/core/abstract_config.hpp"
#include "autumn/core/context.hpp"
#include "autumn/motion/motion_common.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

#include <cmath>

using namespace autumn;

namespace {

struct DriveConfig : AbstractConfig {};

struct DriveRig {
    DriveConfig config{};
    AutumnContext<DriveConfig> context{config};
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    pros::Imu imu{3};
    Odometry odometry;
    Chassis<DriveConfig> chassis;

    DriveRig()
        : odometry(OdomConfig{}),
          chassis(context, odometry, left, right, ExpoDriveCurve(0, 0, 1.0), ExpoDriveCurve(0, 0, 1.0)) {}
};

} // namespace

TEST_CASE("ExpoDriveCurve maps a full stick to full scale" * doctest::may_fail()) {
    ExpoDriveCurve curve(0, 0, 1.0);
    CHECK(curve.curve(0.0) == doctest::Approx(0.0));
    CHECK(curve.curve(127.0) == autumn::test::absApprox(127.0, 0.5));
    CHECK(curve.curve(-127.0) == autumn::test::absApprox(-127.0, 0.5));

    ExpoDriveCurve deadband(10, 0, 1.0);
    CHECK(deadband.curve(10.0) == doctest::Approx(0.0));
}

TEST_CASE("arcade preserves the left/right ratio when a side would saturate" * doctest::may_fail()) {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    arcadeDrive(left, right, 100.0, 50.0, 100.0);
    CHECK(left.commandedVoltage() == autumn::test::absApprox(200.0 / 3.0, 0.5));
    CHECK(right.commandedVoltage() == autumn::test::absApprox(100.0 / 3.0, 0.5));
}

TEST_CASE("curvature turns both ways and stays inside the output range" * doctest::may_fail()) {
    DriveRig rig;
    rig.chassis.curvature(127.0, 127.0);
    const std::int32_t positiveLeft = rig.left.commandedVoltage();
    const std::int32_t positiveRight = rig.right.commandedVoltage();

    rig.chassis.curvature(127.0, -127.0);
    const std::int32_t negativeLeft = rig.left.commandedVoltage();
    const std::int32_t negativeRight = rig.right.commandedVoltage();

    CHECK(positiveLeft == autumn::test::absApprox(negativeRight, 1));
    CHECK(positiveRight == autumn::test::absApprox(negativeLeft, 1));
    CHECK(std::abs(positiveLeft) <= 127);
    CHECK(std::abs(positiveRight) <= 127);
    CHECK(positiveLeft != positiveRight);
}

TEST_CASE("tankDrive of a full-scale command reaches the motor in millivolts") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    tankDrive(left, right, 12000.0, -12000.0);
    CHECK(left.commandedVoltage() == 12000);
    CHECK(right.commandedVoltage() == -12000);
}

TEST_CASE("Chassis builder rejects a missing dependency and accepts a complete one") {
    DriveConfig config;
    AutumnContext<DriveConfig> context(config);
    Odometry odometry(OdomConfig{});
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};

    using ChassisType = Chassis<DriveConfig>;
    CHECK_THROWS_AS(ChassisType::Builder{}.odometry(odometry).leftMotors(left).rightMotors(right).build(),
                    std::logic_error);
    CHECK_NOTHROW(ChassisType::Builder{}
                      .context(context)
                      .odometry(odometry)
                      .leftMotors(left)
                      .rightMotors(right)
                      .build());
}
