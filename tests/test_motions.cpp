#include "doctest.h"

#include "support.hpp"

#include "autumn/control/pid.hpp"
#include "autumn/motion/follow.hpp"
#include "autumn/motion/move_to_point.hpp"
#include "autumn/motion/move_to_pose.hpp"
#include "autumn/motion/turn_to.hpp"

#include "pros/motor_group.hpp"

#include <algorithm>
#include <cmath>
#include <string>

using namespace autumn;
using autumn::test::DiffDriveSim;
using autumn::test::PlantHook;
using autumn::test::simPose;

namespace {

struct ClosedLoop {
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    DiffDriveSim sim;
    PID lateral;
    PID angular;
    MotionSettings settings;
    PlantHook hook;

    ClosedLoop(double lateralP, double angularP)
        : lateral(lateralP, 0.0, 0.0), angular(angularP, 0.0, 0.0), hook(sim) {
        sim.tauSec = 0.03;
        sim.bindDrive(&left, &right);
        settings.poseGetter = [this] { return simPose(sim); };
        settings.leftMotors = &left;
        settings.rightMotors = &right;
        settings.trackWidth = 10.5_in;
        settings.lateralPid = &lateral;
        settings.angularPid = &angular;
        settings.maxVoltage = 12000.0;
        settings.lateralSlew = 1.0e9;
        settings.angularSlew = 1.0e9;
    }
};

int peakVoltage(pros::MotorGroup& left, pros::MotorGroup& right) {
    return std::max(std::abs(left.commandedVoltage()), std::abs(right.commandedVoltage()));
}

} // namespace

TEST_CASE("motions command millivolts rather than the 127 joystick scale") {
    pros::testing::resetClock();
    pros::screen::clearLastText();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    PID lateral(10.0, 0.0, 0.0);
    PID angular(3.0, 0.0, 0.0);
    int peak = 0;
    pros::testing::setDelayHook([&](std::uint32_t) {
        peak = std::max(peak, peakVoltage(left, right));
    });

    MotionSettings settings;
    settings.poseGetter = [] { return Pose(0.0, 0.0, 0.0); };
    settings.leftMotors = &left;
    settings.rightMotors = &right;
    settings.lateralPid = &lateral;
    settings.angularPid = &angular;
    settings.maxVoltage = 127.0;
    settings.lateralSlew = 1.0e9;
    settings.angularSlew = 1.0e9;
    moveToPoint(Pose(0.0, 48.0, 0.0), 50_ms, {}, settings);
    pros::testing::setDelayHook(nullptr);

    CHECK(peak > 6000);
}

TEST_CASE("turnToHeading reaches a clockwise target") {
    ClosedLoop rig(0.0, 20.0);
    TurnToParams params;
    params.earlyExitRange = 8_deg;
    turnToHeading(90_deg, 1500_ms, params, rig.settings);
    CHECK(rig.sim.headingDegrees() == autumn::test::absApprox(90.0, 12.0));
}

TEST_CASE("turnToHeading Auto takes the shorter direction") {
    ClosedLoop rig(0.0, 20.0);
    TurnToParams params;
    params.direction = TurnDirection::Auto;
    params.earlyExitRange = 8_deg;
    turnToHeading(-90_deg, 1500_ms, params, rig.settings);
    CHECK(rig.sim.headingDegrees() == autumn::test::absApprox(-90.0, 12.0));
}

TEST_CASE("turnToHeading reaches a counter-clockwise target") {
    ClosedLoop rig(0.0, 20.0);
    TurnToParams params;
    params.direction = TurnDirection::CounterClockwise;
    params.earlyExitRange = 8_deg;
    turnToHeading(-90_deg, 1500_ms, params, rig.settings);
    CHECK(rig.sim.headingDegrees() == autumn::test::absApprox(-90.0, 12.0));
}

TEST_CASE("moveToPoint arrives on a forward LemLib target") {
    ClosedLoop rig(500.0, 40.0);
    moveToPoint(Pose(0.0, 24.0, 0.0), 2000_ms, {}, rig.settings);
    CHECK(rig.sim.x() == autumn::test::absApprox(0.0, 3.0));
    CHECK(rig.sim.y() == autumn::test::absApprox(24.0, 3.0));
}

TEST_CASE("overshoot backs up instead of turning around") {
    ClosedLoop rig(500.0, 40.0);
    rig.sim.setPose(0.0, 12.0, 0.0);
    moveToPoint(Pose(0.0, 0.0, 0.0), 2000_ms, {}, rig.settings);
    CHECK(rig.sim.y() == autumn::test::absApprox(0.0, 3.0));
    CHECK(std::abs(rig.sim.headingDegrees()) < 25.0);
}

TEST_CASE("reversed moveToPoint drives backward to a point behind the robot") {
    ClosedLoop rig(500.0, 40.0);
    MoveToPointParams params;
    params.reversed = true;
    moveToPoint(Pose(0.0, -24.0, 0.0), 2000_ms, params, rig.settings);
    CHECK(rig.sim.y() < -18.0);
    CHECK(std::abs(rig.sim.headingDegrees()) < 25.0);
}

TEST_CASE("inside 7.5 inches the angular output is off") {
    pros::testing::resetClock();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    PID lateral(400.0, 0.0, 0.0);
    PID angular(80.0, 0.0, 0.0);
    std::int32_t leftVoltage = 0;
    std::int32_t rightVoltage = 0;
    pros::testing::setDelayHook([&](std::uint32_t) {
        leftVoltage = left.commandedVoltage();
        rightVoltage = right.commandedVoltage();
    });

    MotionSettings settings;
    settings.poseGetter = [] { return Pose(0.0, 0.0, 40.0); };
    settings.leftMotors = &left;
    settings.rightMotors = &right;
    settings.lateralPid = &lateral;
    settings.angularPid = &angular;
    settings.maxVoltage = 12000.0;
    settings.lateralSlew = 1.0e9;
    settings.angularSlew = 1.0e9;
    moveToPoint(Pose(0.0, 5.0, 0.0), 40_ms, {}, settings);
    pros::testing::setDelayHook(nullptr);

    CHECK(std::abs(leftVoltage) > 1000);
    CHECK(std::abs(leftVoltage - rightVoltage) < 500);
}

TEST_CASE("minSpeed is a floor on the lateral output") {
    pros::testing::resetClock();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    PID lateral(0.0, 0.0, 0.0);
    PID angular(0.0, 0.0, 0.0);
    int peak = 0;
    pros::testing::setDelayHook([&](std::uint32_t) { peak = std::max(peak, peakVoltage(left, right)); });

    MotionSettings settings;
    settings.poseGetter = [] { return Pose(0.0, 0.0, 0.0); };
    settings.leftMotors = &left;
    settings.rightMotors = &right;
    settings.lateralPid = &lateral;
    settings.angularPid = &angular;
    settings.maxVoltage = 12000.0;
    settings.lateralSlew = 1.0e9;
    settings.angularSlew = 1.0e9;
    MoveToPointParams params;
    params.minLateralSpeed = Number(0.8);
    moveToPoint(Pose(0.0, 36.0, 0.0), 40_ms, params, settings);
    pros::testing::setDelayHook(nullptr);
    CHECK(peak >= 9000);
}

TEST_CASE("timing out is a normal exit and does not raise MotionTimeout") {
    pros::testing::resetClock();
    pros::screen::clearLastText();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    PID lateral(10.0, 0.0, 0.0);
    PID angular(3.0, 0.0, 0.0);
    pros::testing::setDelayHook([&](std::uint32_t) {});
    MotionSettings settings;
    settings.poseGetter = [] { return Pose(0.0, 0.0, 0.0); };
    settings.leftMotors = &left;
    settings.rightMotors = &right;
    settings.lateralPid = &lateral;
    settings.angularPid = &angular;
    settings.lateralSlew = 1.0e9;
    settings.angularSlew = 1.0e9;
    moveToPoint(Pose(0.0, 48.0, 0.0), 30_ms, {}, settings);
    pros::testing::setDelayHook(nullptr);
    CHECK(pros::screen::lastText().empty());
}

TEST_CASE("moveToPose is boomerang and finishes on the target heading") {
    ClosedLoop rig(500.0, 40.0);
    moveToPose(Pose(24.0, 24.0, 90.0), 2500_ms, {}, rig.settings);
    CHECK(rig.sim.x() == autumn::test::absApprox(24.0, 4.0));
    CHECK(rig.sim.y() == autumn::test::absApprox(24.0, 4.0));
    CHECK(rig.sim.headingDegrees() == autumn::test::absApprox(90.0, 15.0));
}

TEST_CASE("pure pursuit stays near the path instead of cutting the corner") {
    ClosedLoop rig(400.0, 35.0);
    const std::string body = "0,0,80\n0,24,80\n24,24,0\n";
    asset path{reinterpret_cast<std::uint8_t*>(const_cast<char*>(body.data())), body.size()};
    double closestCorner = 1.0e9;
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
        rig.sim.step(milliseconds);
        const double dx = rig.sim.x() - 0.0;
        const double dy = rig.sim.y() - 24.0;
        closestCorner = std::min(closestCorner, std::hypot(dx, dy));
    });
    follow(path, 8_in, 4000_ms, false, {}, rig.settings);
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) { rig.sim.step(milliseconds); });
    CHECK(closestCorner < 6.0);
    CHECK(std::hypot(rig.sim.x() - 24.0, rig.sim.y() - 24.0) < 6.0);
}
