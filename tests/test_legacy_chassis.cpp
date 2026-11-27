#include "support.hpp"
#include "doctest.h"

#include "autumn/chassis/chassis.hpp"
#include "autumn/chassis/legacy_chassis.hpp"
#include "autumn/core/abstract_config.hpp"
#include "autumn/core/context.hpp"
#include "autumn/localization/odometry.hpp"
#include "autumn/motion/motion_common.hpp"

#include "pros/motor_group.hpp"

#include <cmath>
#include <string>

using namespace autumn;
using autumn::test::DiffDriveSim;

namespace {

// Drives a DiffDriveSim and feeds an Odometry from the same sensors, so the
// LegacyChassis poseGetter sees the plant the motors are moving.
struct LegacyRig {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    pros::Imu imu{3};
    pros::Rotation vertical{4};
    DiffDriveSim sim;
    TrackingWheel verticalWheel{&vertical, 2.75_in, 0_in};
    Odometry odometry;
    LegacyChassis chassis;

    LegacyRig()
        : odometry(makeConfig()),
          chassis(LegacyDrivetrain{&left, &right, 10.5, 4.0, 2.0},
                  ControllerSettings{10.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
                  ControllerSettings{3.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, OdomSensorSet{}, odometry) {
        sim.tauSec = 0.03;
        sim.bindDrive(&left, &right);
        sim.bindImu(&imu);
        sim.bindVertical(&vertical, 2.75, 0.0);
        sim.setPose(0.0, 0.0, 0.0);
        odometry.update();
    }

    OdomConfig makeConfig() {
        OdomConfig config;
        config.imu = &imu;
        config.verticalWheel = &verticalWheel;
        config.trackWidth = 10.5_in;
        return config;
    }

    void tick(std::uint32_t milliseconds) {
        sim.step(milliseconds);
        odometry.update();
    }
};

struct ScaleConfig : AbstractConfig {};

} // namespace

TEST_CASE("LegacyChassis drives to a point through the sim plant") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    LegacyRig rig;
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) { rig.tick(milliseconds); });
    rig.chassis.moveToPoint(Pose(0.0, 18.0, 0.0), 3000_ms, {}, false);
    pros::testing::setDelayHook(nullptr);
    CHECK(rig.sim.y() > 15.0);
    CHECK(std::abs(autumn::test::headingDiffDeg(rig.sim.headingDegrees(), 0.0)) < 15.0);
}

TEST_CASE("a blocking LegacyChassis motion runs after a previous cancel") {
    // cancelMotion() leaves the global flag set. The blocking path never cleared
    // it, so the next call returned immediately without moving.
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    LegacyRig rig;
    rig.chassis.cancelMotion();
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) { rig.tick(milliseconds); });
    const std::uint32_t started = pros::millis();
    rig.chassis.moveToPoint(Pose(0.0, 18.0, 0.0), 3000_ms, {}, false);
    const std::uint32_t elapsed = pros::millis() - started;
    pros::testing::setDelayHook(nullptr);
    CHECK(elapsed > 100);
    CHECK(rig.sim.y() > 15.0);
}

TEST_CASE("LegacyChassis converts its ControllerSettings slew into millivolts") {
    // ControllerSettings::slew is a LemLib 0-127 quantity. Ignoring it let every
    // motion command full power on the first tick.
    pros::testing::resetClock();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    DiffDriveSim sim;
    sim.tauSec = 0.03;
    sim.bindDrive(&left, &right);
    Odometry odometry(OdomConfig{});
    LegacyChassis chassis(LegacyDrivetrain{&left, &right, 10.5, 4.0, 2.0},
                          ControllerSettings{50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
                          ControllerSettings{3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, OdomSensorSet{},
                          odometry);
    std::int32_t firstCommand = -1;
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
        sim.step(milliseconds);
        if (firstCommand < 0) {
            firstCommand = left.commandedVoltage();
        }
    });
    chassis.moveToPoint(Pose(0.0, 400.0, 0.0), 200_ms, {}, false);
    pros::testing::setDelayHook(nullptr);
    // A slew of 1 on the 0-127 scale is ~94 mV once expressed in millivolts.
    CHECK(firstCommand >= 0);
    CHECK(firstCommand < 500);
}

TEST_CASE("LegacyChassis honours the smallError settle window") {
    // smallError/largeError and their timeouts were accepted and dropped. A wide
    // smallError lets the motion settle short of the target instead of driving to
    // the 1 in default window.
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    struct SettingsCase {
        double smallError;
        double toleranceIn;
    };
    for (const SettingsCase& testCase : {SettingsCase{0.0, 1.5}, SettingsCase{4.0, 5.0}}) {
        pros::MotorGroup left{1};
        pros::MotorGroup right{2};
        pros::Imu imu{3};
        pros::Rotation vertical{4};
        DiffDriveSim sim;
        TrackingWheel verticalWheel{&vertical, 2.75_in, 0_in};
        sim.tauSec = 0.03;
        sim.bindDrive(&left, &right);
        sim.bindImu(&imu);
        sim.bindVertical(&vertical, 2.75, 0.0);
        sim.setPose(0.0, 0.0, 0.0);
        OdomConfig config;
        config.imu = &imu;
        config.verticalWheel = &verticalWheel;
        config.trackWidth = 10.5_in;
        Odometry odometry(config);
        odometry.update();
        LegacyChassis chassis(
            LegacyDrivetrain{&left, &right, 10.5, 4.0, 2.0},
            ControllerSettings{10.0, 0.0, 0.03, 0.0, testCase.smallError, 0.0, 0.0, 0.0, 0.0},
            ControllerSettings{3.0, 0.0, 0.2, 0.0, testCase.smallError, 0.0, 0.0, 0.0, 0.0}, OdomSensorSet{},
            odometry);

        CAPTURE(testCase.smallError);
        CHECK(to_inches(chassis.settings().lateralSettleError) == autumn::test::absApprox(
                  testCase.smallError > 0.0 ? testCase.smallError : 1.0, 1e-6));

        pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
            sim.step(milliseconds);
            odometry.update();
        });
        moveToPose(Pose(0.0, 20.0, 0.0), 3000_ms, {}, chassis.settings());
        pros::testing::setDelayHook(nullptr);
        CHECK(std::hypot(sim.x(), sim.y() - 20.0) < testCase.toleranceIn);
    }
}

TEST_CASE("Chassis motion settings run in millivolts, not a normalized unit") {
    // The struct default was switched to 1.0 while Chassis never set it, so every
    // Chassis motion clamps to full power for the whole move.
    pros::testing::resetClock();
    setMotionCancelled(false);
    ScaleConfig config;
    AutumnContext<ScaleConfig> context(config);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    DiffDriveSim sim;
    sim.tauSec = 0.03;
    sim.bindDrive(&left, &right);
    Odometry odometry(OdomConfig{});
    Chassis<ScaleConfig> chassis(context, odometry, left, right);

    const MotionSettings settings = chassis.motionSettings();
    CHECK(settings.maxVoltage == 12000.0);

    int saturatedTicks = 0;
    int totalTicks = 0;
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
        sim.step(milliseconds);
        ++totalTicks;
        if (std::abs(left.commandedVoltage()) >= 11990) {
            ++saturatedTicks;
        }
    });
    moveToPoint(Pose(0.0, 36.0, 0.0), 2000_ms, {}, settings);
    pros::testing::setDelayHook(nullptr);
    // A proportional controller must spend part of the move below full scale.
    CHECK(saturatedTicks < totalTicks);
    CHECK(sim.y() > 30.0);
}
