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

TEST_CASE("ExpoDriveCurve maps a full stick to full scale") {
    ExpoDriveCurve curve(0, 0, 1.0);
    CHECK(curve.curve(0.0) == doctest::Approx(0.0));
    CHECK(curve.curve(127.0) == autumn::test::absApprox(127.0, 0.5));
    CHECK(curve.curve(-127.0) == autumn::test::absApprox(-127.0, 0.5));

    ExpoDriveCurve deadband(10, 0, 1.0);
    CHECK(deadband.curve(10.0) == doctest::Approx(0.0));
}

TEST_CASE("ExpoDriveCurve deliberately diverges from the LemLib 0.5.6 shape") {
    // This is documented as intentional in docs/index.md. LemLib's curve stays
    // near minOutput even at full deflection; Autumn maps full stick to full
    // scale. The test pins the divergence so it cannot change silently, and pins
    // the properties a driver actually relies on: monotonic, and continuous at
    // the deadband edge.
    ExpoDriveCurve curve(3, 10, 1.019);
    CHECK(curve.curve(127.0) == autumn::test::absApprox(127.0, 0.01));
    // LemLib 0.5.6 with the same arguments returns about 10.9 here.
    CHECK(curve.curve(127.0) > 60.0);
    CHECK(curve.curve(3.0) == doctest::Approx(0.0));
    double previous = 0.0;
    for (double input = 3.0; input <= 127.0; input += 1.0) {
        const double output = curve.curve(input);
        CHECK(output >= previous);
        previous = output;
    }
    // Jump straight past the deadband lands near minOutput, not at full scale.
    CHECK(curve.curve(4.0) < 20.0);
    CHECK(curve.curve(4.0) >= 10.0);
}

TEST_CASE("arcade preserves the left/right ratio when a side would saturate") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    arcadeDrive(left, right, 100.0, 50.0, 100.0);
    CHECK(left.commandedVoltage() == autumn::test::absApprox(12000.0, 50.0));
    CHECK(right.commandedVoltage() == autumn::test::absApprox(4000.0, 50.0));
}

TEST_CASE("curvature turns both ways and stays inside the output range") {
    DriveRig rig;
    rig.chassis.curvature(127.0, 127.0);
    const std::int32_t positiveLeft = rig.left.commandedVoltage();
    const std::int32_t positiveRight = rig.right.commandedVoltage();

    rig.chassis.curvature(127.0, -127.0);
    const std::int32_t negativeLeft = rig.left.commandedVoltage();
    const std::int32_t negativeRight = rig.right.commandedVoltage();

    CHECK(positiveLeft == autumn::test::absApprox(negativeRight, 1));
    CHECK(positiveRight == autumn::test::absApprox(negativeLeft, 1));
    CHECK(std::abs(positiveLeft) <= 12000);
    CHECK(std::abs(positiveRight) <= 12000);
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
