#include "support.hpp"
#include "doctest.h"

#include "autumn/hardware/imu.hpp"
#include "autumn/localization/odometry.hpp"
#include "autumn/localization/tracking_wheel.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

#include <cmath>

using namespace autumn;

namespace {

std::int32_t centidegreesForInches(double inches, double diameterIn) {
    const double revolutions = inches / (M_PI * diameterIn);
    return static_cast<std::int32_t>(std::lround(revolutions * 360.0 * 100.0));
}

double motorDegreesForInches(double inches, double diameterIn) {
    const double revolutions = inches / (M_PI * diameterIn);
    return revolutions * 360.0;
}

} // namespace

TEST_CASE("setPose round trip") {
    Odometry odometry(OdomConfig{});
    odometry.setPose(Pose(3.0, -4.0, 90.0));
    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.x) == doctest::Approx(3.0));
    CHECK(to_inches(pose.y) == doctest::Approx(-4.0));
    CHECK(to_degrees(pose.theta) == autumn::test::absApprox(90.0, 1e-6));
}

TEST_CASE("vertical wheel plus IMU at heading 0 reports +y") {
    pros::Imu imu(1);
    pros::Rotation vertical(2);
    TrackingWheel wheel(&vertical, 2.75_in, 0_in);
    OdomConfig config;
    config.imu = &imu;
    config.verticalWheel = &wheel;
    config.trackWidth = 10.5_in;
    Odometry odometry(config);

    imu.set_rotation(0.0);
    vertical.setCentidegrees(0);
    odometry.update();
    vertical.setCentidegrees(centidegreesForInches(10.0, 2.75));
    odometry.update();

    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.x) == autumn::test::absApprox(0.0, 0.25));
    CHECK(to_inches(pose.y) == autumn::test::absApprox(10.0, 0.25));
    CHECK(to_degrees(pose.theta) == autumn::test::absApprox(0.0, 0.5));
}

TEST_CASE("odometry pose changes when the vertical wheel moves") {
    pros::Imu imu(1);
    pros::Rotation vertical(2);
    TrackingWheel wheel(&vertical, 2.75_in, 0_in);
    OdomConfig config;
    config.imu = &imu;
    config.verticalWheel = &wheel;
    Odometry odometry(config);
    imu.set_rotation(0.0);
    vertical.setCentidegrees(0);
    odometry.update();
    vertical.setCentidegrees(centidegreesForInches(10.0, 2.75));
    odometry.update();
    CHECK(to_inches(odometry.getPose().distance(Pose{})) > 1.0);
}

TEST_CASE("forward travel at heading 90 goes to +x") {
    pros::Imu imu(1);
    pros::Rotation vertical(2);
    TrackingWheel wheel(&vertical, 2.75_in, 0_in);
    OdomConfig config;
    config.imu = &imu;
    config.verticalWheel = &wheel;
    Odometry odometry(config);
    odometry.setPose(Pose(0.0, 0.0, 90.0));
    imu.set_rotation(90.0);
    vertical.setCentidegrees(0);
    odometry.update();
    vertical.setCentidegrees(centidegreesForInches(10.0, 2.75));
    odometry.update();

    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.x) == autumn::test::absApprox(10.0, 0.25));
    CHECK(to_inches(pose.y) == autumn::test::absApprox(0.0, 0.25));
    CHECK(to_degrees(pose.theta) == autumn::test::absApprox(90.0, 0.5));
}

TEST_CASE("horizontal wheel is strafe, including while an IMU is connected") {
    pros::Imu imu(1);
    pros::Rotation vertical(2);
    pros::Rotation horizontal(3);
    TrackingWheel verticalWheel(&vertical, 2.75_in, 0_in);
    TrackingWheel horizontalWheel(&horizontal, 2.75_in, 0_in);
    OdomConfig config;
    config.imu = &imu;
    config.verticalWheel = &verticalWheel;
    config.horizontalWheel = &horizontalWheel;
    config.trackWidth = 10.5_in;
    Odometry odometry(config);

    imu.set_rotation(0.0);
    vertical.setCentidegrees(0);
    horizontal.setCentidegrees(0);
    odometry.update();
    vertical.setCentidegrees(centidegreesForInches(10.0, 2.75));
    horizontal.setCentidegrees(centidegreesForInches(4.0, 2.75));
    odometry.update();

    // LemLib 0.5.6: +localX contributes -cos(heading) to global x.
    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.x) == autumn::test::absApprox(-4.0, 0.35));
    CHECK(to_inches(pose.y) == autumn::test::absApprox(10.0, 0.35));
}

TEST_CASE("a single horizontal wheel is not a heading source") {
    pros::Rotation horizontal(3);
    TrackingWheel horizontalWheel(&horizontal, 2.75_in, 0_in);
    OdomConfig config;
    config.horizontalWheel = &horizontalWheel;
    config.trackWidth = 10.5_in;
    Odometry odometry(config);
    horizontal.setCentidegrees(0);
    odometry.update();
    horizontal.setCentidegrees(centidegreesForInches(10.0, 2.75));
    odometry.update();
    CHECK(to_degrees(odometry.getPose().theta) == autumn::test::absApprox(0.0, 0.5));
}

TEST_CASE("IME-only straight drive tracks +y") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    OdomConfig config;
    config.leftMotors = &left;
    config.rightMotors = &right;
    config.wheelDiameter = 4_in;
    config.trackWidth = 10.5_in;
    Odometry odometry(config);
    left.setPositionDegrees(0.0);
    right.setPositionDegrees(0.0);
    odometry.update();
    const double degrees = motorDegreesForInches(10.0, 4.0);
    left.setPositionDegrees(degrees);
    right.setPositionDegrees(degrees);
    odometry.update();

    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.y) == autumn::test::absApprox(10.0, 0.35));
    CHECK(to_inches(pose.x) == autumn::test::absApprox(0.0, 0.35));
    CHECK(to_degrees(pose.theta) == autumn::test::absApprox(0.0, 0.5));
}

TEST_CASE("IME point turn with the left side forward is clockwise") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    OdomConfig config;
    config.leftMotors = &left;
    config.rightMotors = &right;
    config.wheelDiameter = 4_in;
    config.trackWidth = 10.5_in;
    Odometry odometry(config);
    left.setPositionDegrees(0.0);
    right.setPositionDegrees(0.0);
    odometry.update();
    const double degrees = motorDegreesForInches(5.0, 4.0);
    left.setPositionDegrees(degrees);
    right.setPositionDegrees(-degrees);
    odometry.update();

    CHECK(to_degrees(odometry.getPose().theta) > 20.0);
    CHECK(to_inches(odometry.getPose().distance(Pose{})) < 1.0);
}

TEST_CASE("IMU heading still compensates a vertically offset wheel") {
    pros::Imu imu(1);
    pros::Rotation vertical(2);
    constexpr double kOffsetIn = 2.0;
    TrackingWheel wheel(&vertical, 2.75_in, inches(kOffsetIn));
    OdomConfig config;
    config.imu = &imu;
    config.verticalWheel = &wheel;
    Odometry odometry(config);
    imu.set_rotation(0.0);
    vertical.setCentidegrees(0);
    odometry.update();

    // Arc of a wheel 2 in to the side through a clockwise quarter turn.
    const double arc = -kOffsetIn * (M_PI / 2.0);
    imu.set_rotation(90.0);
    vertical.setCentidegrees(centidegreesForInches(arc, 2.75));
    odometry.update();

    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.distance(Pose{})) < 0.75);
    CHECK(to_degrees(pose.theta) == autumn::test::absApprox(90.0, 0.5));
}

TEST_CASE("successful IMU calibration is not reported as a failure") {
    pros::testing::resetClock();
    pros::screen::clearLastText();
    pros::Imu imu(1);
    imu.setCalibrating(false);
    OdomConfig config;
    config.imu = &imu;
    Odometry odometry(config);
    odometry.calibrateImu();
    CHECK(pros::screen::lastText().empty());
}

TEST_CASE("odometry task starts and stops") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    Odometry odometry(OdomConfig{});
    odometry.start();
    pros::delay(20);
    odometry.stop();
    odometry.setPose(Pose(1.0, 2.0, 0.0));
    CHECK(to_inches(odometry.getPose().x) == doctest::Approx(1.0));
}

TEST_CASE("IMU heading wrap at 0/360 does not lose travel") {
    // The IMU reports 0..360, so stepping from 355 to 5 looks like -350 deg. The
    // chord factor then collapses and almost all of the translation is dropped.
    struct Case {
        double startDeg;
        double endDeg;
        double expectedInches;
    };
    const Case cases[] = {{355.0, 5.0, 10.0}, {5.0, 355.0, -10.0}, {40.0, 50.0, 10.0}};
    for (const Case& testCase : cases) {
        pros::Imu imu(1);
        pros::Rotation vertical(2);
        TrackingWheel wheel(&vertical, 2.75_in, 0_in);
        OdomConfig config;
        config.imu = &imu;
        config.verticalWheel = &wheel;
        config.trackWidth = 10.5_in;
        Odometry odometry(config);

        imu.set_rotation(testCase.startDeg);
        vertical.setCentidegrees(0);
        odometry.update();
        odometry.setPose(Pose(0.0, 0.0, testCase.startDeg));

        vertical.setCentidegrees(centidegreesForInches(10.0, 2.75));
        imu.set_rotation(testCase.endDeg);
        odometry.update();

        CAPTURE(testCase.startDeg);
        const Pose pose = odometry.getPose();
        CHECK(std::hypot(to_inches(pose.x), to_inches(pose.y)) == autumn::test::absApprox(10.0, 0.35));
        CHECK(std::hypot(to_inches(pose.x), to_inches(pose.y)) > 9.0);
        (void)testCase.expectedInches;
    }
}

TEST_CASE("odometry updateCount increments each tick") {
    pros::Imu imu(1);
    OdomConfig config;
    config.imu = &imu;
    Odometry odometry(config);
    imu.set_rotation(0.0);
    CHECK(odometry.updateCount() == 0);
    odometry.update();
    CHECK(odometry.updateCount() == 1);
    odometry.update();
    CHECK(odometry.updateCount() == 2);
}

TEST_CASE("wheel delta glitches are rejected and do not jump the pose") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    OdomConfig config;
    config.leftMotors = &left;
    config.rightMotors = &right;
    config.wheelDiameter = 4_in;
    config.trackWidth = 10.5_in;
    Odometry odometry(config);
    left.setPositionDegrees(0.0);
    right.setPositionDegrees(0.0);
    odometry.update();
    left.setPositionDegrees(10000.0);
    right.setPositionDegrees(10000.0);
    odometry.update();
    CHECK(to_inches(odometry.getPose().distance(Pose{})) < 2.0);
}

TEST_CASE("heading keeps accumulating past a full turn with an IMU") {
    pros::Imu imu(1);
    pros::Rotation vertical(2);
    TrackingWheel wheel(&vertical, 2.75_in, 0_in);
    OdomConfig config;
    config.imu = &imu;
    config.verticalWheel = &wheel;
    Odometry odometry(config);

    imu.set_rotation(0.0);
    vertical.setCentidegrees(0);
    odometry.update();
    // Spin through a full turn in 10 deg steps; the pose must stay put.
    for (int step = 1; step <= 36; ++step) {
        imu.set_rotation(step * 10.0);
        odometry.update();
    }
    const Pose pose = odometry.getPose();
    CHECK(to_inches(pose.distance(Pose{})) < 0.5);
    CHECK(std::abs(autumn::test::headingDiffDeg(to_degrees(pose.theta), 0.0)) < 1.0);
}
