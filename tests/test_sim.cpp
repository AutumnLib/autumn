#include "doctest.h"

#include "support.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <cmath>

using autumn::test::DiffDriveSim;
using autumn::test::PlantHook;

TEST_CASE("full voltage drives forward in +y") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    DiffDriveSim sim;
    sim.tauSec = 0.02;
    sim.bindDrive(&left, &right);
    PlantHook hook(sim);

    left.move_voltage(12000);
    right.move_voltage(12000);
    for (int i = 0; i < 100; ++i) {
        pros::delay(10);
    }

    const double freeIps = (600.0 / 60.0) * M_PI * 4.0;
    CHECK(sim.y() > freeIps * 0.85);
    CHECK(sim.x() == autumn::test::absApprox(0.0, 1.0));
    CHECK(sim.headingDegrees() == autumn::test::absApprox(0.0, 2.0));
}

TEST_CASE("left wheel forward and right wheel back turns clockwise") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    DiffDriveSim sim;
    sim.tauSec = 0.02;
    sim.bindDrive(&left, &right);
    PlantHook hook(sim);

    left.move_voltage(12000);
    right.move_voltage(-12000);
    for (int i = 0; i < 40; ++i) {
        pros::delay(10);
    }

    CHECK(sim.headingDegrees() > 20.0);
    CHECK(sim.x() == autumn::test::absApprox(0.0, 2.0));
    CHECK(sim.y() == autumn::test::absApprox(0.0, 2.0));
}

TEST_CASE("offset tracking wheel measures the arc of a point turn") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    pros::Rotation vertical(3);
    DiffDriveSim sim;
    sim.tauSec = 0.01;
    sim.bindDrive(&left, &right);
    sim.bindVertical(&vertical, 2.75, 2.0);
    PlantHook hook(sim);

    left.move_voltage(12000);
    right.move_voltage(-12000);
    for (int i = 0; i < 80; ++i) {
        pros::delay(10);
    }

    const double expected = -2.0 * sim.headingRadians();
    CHECK(sim.verticalInches() == autumn::test::absApprox(expected, 0.5));
}

TEST_CASE("sensor noise is optional and deterministic") {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    DiffDriveSim quiet;
    quiet.tauSec = 0.02;
    quiet.bindDrive(&left, &right);
    left.move_voltage(12000);
    right.move_voltage(12000);
    for (int i = 0; i < 30; ++i) {
        quiet.step(10);
    }

    DiffDriveSim again;
    again.tauSec = 0.02;
    again.bindDrive(&left, &right);
    for (int i = 0; i < 30; ++i) {
        again.step(10);
    }
    CHECK(again.y() == doctest::Approx(quiet.y()));

    DiffDriveSim noisy;
    noisy.tauSec = 0.02;
    noisy.noiseIn = 0.25;
    noisy.bindDrive(&left, &right);
    for (int i = 0; i < 30; ++i) {
        noisy.step(10);
    }
    // Noise is applied to the tracking-wheel travel, not the true pose.
    CHECK(noisy.verticalInches() != autumn::test::absApprox(quiet.verticalInches(), 0.05));
}
