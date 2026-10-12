#include "support.hpp"
#include "doctest.h"

#include "autumn/hardware/adi_encoder.hpp"
#include "autumn/hardware/encoder.hpp"
#include "autumn/hardware/imu.hpp"
#include "autumn/hardware/motor_group.hpp"
#include "autumn/localization/tracking_wheel.hpp"

#include "pros/adi.hpp"
#include "pros/error.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <cmath>

using namespace autumn;

TEST_CASE("normalized motor power is full-scale millivolts") {
    pros::MotorGroup raw{1};
    hardware::MotorGroup motors(&raw);
    CHECK(motors.move(Number(1.0)) == 1);
    CHECK(raw.commandedVoltage() == 12000);
    motors.move(Number(-0.5));
    CHECK(raw.commandedVoltage() == -6000);
    motors.move(Number(4.0));
    CHECK(raw.commandedVoltage() == 12000);
}

TEST_CASE("rotation sensor centidegrees become radians, and a reconnect does not jump") {
    pros::testing::resetClock();
    pros::screen::clearLastText();
    pros::Rotation raw(1);
    hardware::V5RotationEncoder encoder(&raw);

    raw.setCentidegrees(0);
    CHECK(to_degrees(encoder.getAngle()) == autumn::test::absApprox(0.0, 1e-4));

    raw.setCentidegrees(18000);
    CHECK(to_degrees(encoder.getAngle()) == autumn::test::absApprox(180.0, 1e-3));

    raw.setConnected(false);
    CHECK(to_degrees(encoder.getAngle()) == autumn::test::absApprox(180.0, 1e-3));
    CHECK(pros::screen::lastText() == "E002");
    pros::screen::clearLastText();
    encoder.getAngle();
    CHECK(pros::screen::lastText().empty());

    raw.setConnected(true);
    raw.setCentidegrees(18000);
    CHECK(to_degrees(encoder.getAngle()) == autumn::test::absApprox(180.0, 1e-3));
    raw.setCentidegrees(27000);
    CHECK(to_degrees(encoder.getAngle()) == autumn::test::absApprox(270.0, 1e-3));
    CHECK(encoder.isConnected());
}

TEST_CASE("ADI encoder ticks are 360 per revolution") {
    hardware::AdiEncoder encoder('A', 'B', false);
    pros::adi::Encoder* raw = pros::adi::Encoder::lastCreated();
    REQUIRE(raw != nullptr);
    raw->setTicks(0);
    encoder.getAngle();
    raw->setTicks(360);
    CHECK(to_radians(encoder.getAngle()) == autumn::test::absApprox(2.0 * M_PI, 1e-3));
}

TEST_CASE("tracking wheel distance is angle times diameter over two") {
    pros::Rotation raw(2);
    hardware::V5RotationEncoder encoder(&raw);
    TrackingWheel wheel(&encoder, 2.75_in, 1.5_in);

    raw.setCentidegrees(0);
    encoder.getAngle();
    const double oneRevCentideg = 360.0 * 100.0;
    raw.setCentidegrees(static_cast<std::int32_t>(oneRevCentideg));
    const double circumference = M_PI * 2.75;
    CHECK(to_inches(wheel.readDistance()) == autumn::test::absApprox(circumference, 1e-3));

    wheel.readAndResetDelta();
    raw.setCentidegrees(static_cast<std::int32_t>(oneRevCentideg + 180.0 * 100.0));
    const Length delta = wheel.readAndResetDelta(90_deg);
    const double expected = circumference / 2.0 - 1.5 * (M_PI / 2.0);
    CHECK(to_inches(delta) == autumn::test::absApprox(expected, 1e-3));
}

TEST_CASE("IMU heading is clockwise degrees times the gyro scalar, and calibration can time out") {
    pros::testing::resetClock();
    pros::Imu raw(10);
    hardware::V5IMU imu(&raw, Number(2.0));
    raw.set_rotation(90.0);
    CHECK(to_degrees(imu.getRotation()) == autumn::test::absApprox(180.0, 1e-3));

    raw.setCalibrating(true);
    CHECK(imu.calibrate(true) == PROS_ERR);
    CHECK(pros::millis() > 5000);

    raw.setCalibrating(false);
    pros::testing::resetClock();
    CHECK(imu.calibrate(true) == 1);
}
