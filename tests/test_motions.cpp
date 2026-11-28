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

// --- Regressions for the legacy-MVP review -----------------------------------

namespace {

// Quarter circle of radius `radiusIn` from the origin, curving toward +x.
std::string arcPath(double radiusIn, int steps) {
    std::string body;
    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / steps;
        const double angle = t * M_PI / 2.0;
        body += std::to_string(radiusIn * (1.0 - std::cos(angle))) + ", " +
                std::to_string(radiusIn * std::sin(angle)) + ", 80\n";
    }
    body += "endData\n";
    return body;
}

} // namespace

TEST_CASE("turnToHeading finishes when a one-way turn direction is requested") {
    // The directional error used to grow by a full turn on arrival, so the exit
    // test never fired and the robot circled until the timeout.
    for (TurnDirection direction : {TurnDirection::Clockwise, TurnDirection::CounterClockwise}) {
        ClosedLoop rig(0.0, 20.0);
        TurnToParams params;
        params.direction = direction;
        params.earlyExitRange = 3_deg;
        const std::uint32_t started = pros::millis();
        turnToHeading(90_deg, 1500_ms, params, rig.settings);
        const std::uint32_t elapsed = pros::millis() - started;
        CHECK(elapsed < 1500);
        CHECK(std::abs(autumn::test::headingDiffDeg(rig.sim.headingDegrees(), 90.0)) < 12.0);
    }
}

TEST_CASE("swingToHeading finishes when a one-way turn direction is requested") {
    ClosedLoop rig(0.0, 20.0);
    TurnToParams params;
    params.direction = TurnDirection::Clockwise;
    params.earlyExitRange = 3_deg;
    const std::uint32_t started = pros::millis();
    swingToHeading(90_deg, SwingSide::Right, 1500_ms, params, rig.settings);
    CHECK(pros::millis() - started < 1500);
    CHECK(std::abs(autumn::test::headingDiffDeg(rig.sim.headingDegrees(), 90.0)) < 12.0);
}

TEST_CASE("reversed follow tracks a curved path instead of driving away") {
    // Rotating the pose for a backwards path without swapping the drive sides
    // inverted the steering, and the robot left the field.
    ClosedLoop rig(400.0, 35.0);
    rig.sim.setPose(0.0, 0.0, 180.0);
    const std::string body = arcPath(24.0, 24);
    asset path{reinterpret_cast<std::uint8_t*>(const_cast<char*>(body.data())), body.size()};
    double worstRadialError = 0.0;
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
        rig.sim.step(milliseconds);
        const double radius = std::hypot(rig.sim.x() - 24.0, rig.sim.y());
        worstRadialError = std::max(worstRadialError, std::abs(radius - 24.0));
    });
    follow(path, 8_in, 4000_ms, true, {}, rig.settings);
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) { rig.sim.step(milliseconds); });
    CHECK(worstRadialError < 8.0);
    CHECK(std::hypot(rig.sim.x() - 24.0, rig.sim.y() - 24.0) < 6.0);
}

TEST_CASE("reversed and forward follow agree on the same path") {
    struct Result {
        double x;
        double y;
    };
    auto drive = [](bool reversed) {
        ClosedLoop rig(400.0, 35.0);
        rig.sim.setPose(0.0, 0.0, reversed ? 180.0 : 0.0);
        const std::string body = arcPath(24.0, 24);
        asset path{reinterpret_cast<std::uint8_t*>(const_cast<char*>(body.data())), body.size()};
        pros::testing::setDelayHook([&](std::uint32_t milliseconds) { rig.sim.step(milliseconds); });
        follow(path, 8_in, 4000_ms, reversed, {}, rig.settings);
        pros::testing::setDelayHook(nullptr);
        return Result{rig.sim.x(), rig.sim.y()};
    };
    const Result forward = drive(false);
    const Result backward = drive(true);
    CHECK(std::hypot(forward.x - backward.x, forward.y - backward.y) < 8.0);
}

TEST_CASE("moveToPose reaches axis-aligned targets without timing out") {
    // The exit test demanded distance and heading at once while the settle branch
    // aimed the nose at the final heading, so the lateral term projected to zero
    // and the robot sat still until the timeout.
    struct Target {
        double x;
        double y;
        double heading;
    };
    const Target targets[] = {{0.0, 12.0, 90.0}, {12.0, 0.0, 0.0}, {0.0, 36.0, 90.0}, {0.0, 12.0, 180.0}};
    for (const Target& target : targets) {
        ClosedLoop rig(10.0, 3.0);
        const std::uint32_t started = pros::millis();
        moveToPose(Pose(target.x, target.y, target.heading), 4000_ms, {}, rig.settings);
        CAPTURE(target.x);
        CAPTURE(target.y);
        CAPTURE(target.heading);
        CHECK(pros::millis() - started < 4000);
        CHECK(std::hypot(rig.sim.x() - target.x, rig.sim.y() - target.y) < 2.0);
        CHECK(std::abs(autumn::test::headingDiffDeg(rig.sim.headingDegrees(), target.heading)) < 12.0);
    }
}

TEST_CASE("moveToPose falls back to the default drift when the param is unset") {
    // 0 is the LemLib "use the drivetrain value" sentinel. Taken literally it
    // zeroed the slip cap and the robot never moved.
    ClosedLoop rig(10.0, 3.0);
    MoveToPoseParams params;
    params.horizontalDrift = 0.0;
    moveToPose(Pose(0.0, 24.0, 0.0), 3000_ms, params, rig.settings);
    CHECK(rig.sim.y() > 20.0);
}

TEST_CASE("follow slews its target speed instead of stepping it") {
    // LemLib ramps the path speed through the lateral slew rate. Without the ramp
    // a corner's speed drop arrives as a step.
    ClosedLoop rig(400.0, 35.0);
    rig.settings.lateralSlew = 200.0;
    const std::string body = "0,0,80\n0,24,80\n0,24,20\n0,48,20\nendData\n";
    asset path{reinterpret_cast<std::uint8_t*>(const_cast<char*>(body.data())), body.size()};
    int largestStep = 0;
    std::int32_t previous = 0;
    pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
        rig.sim.step(milliseconds);
        const std::int32_t current = rig.left.commandedVoltage();
        if (previous != 0) {
            largestStep = std::max(largestStep, std::abs(current - previous));
        }
        previous = current;
    });
    follow(path, 8_in, 4000_ms, false, {}, rig.settings);
    pros::testing::setDelayHook(nullptr);
    // One tick may not exceed the slew step plus the curvature's own share.
    CHECK(largestStep < 2500);
}

TEST_CASE("min lateral speed with chaining leaves voltage on after moveToPose") {
    pros::testing::resetClock();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    PID lateral(0.0, 0.0, 0.0);
    PID angular(0.0, 0.0, 0.0);
    std::int32_t leftVoltage = 0;
    pros::testing::setDelayHook([&](std::uint32_t) { leftVoltage = left.commandedVoltage(); });
    MotionSettings settings;
    settings.poseGetter = [] { return Pose(0.0, 0.0, 0.0); };
    settings.leftMotors = &left;
    settings.rightMotors = &right;
    settings.lateralPid = &lateral;
    settings.angularPid = &angular;
    settings.maxVoltage = 12000.0;
    settings.lateralSlew = 1.0e9;
    settings.angularSlew = 1.0e9;
    MoveToPoseParams params;
    params.minLateralSpeed = Number(0.5);
    moveToPose(Pose(0.0, 6.0, 90.0), 30_ms, params, settings);
    pros::testing::setDelayHook(nullptr);
    CHECK(std::abs(leftVoltage) >= 5000);
}

TEST_CASE("stall exit ends a motion when the robot stops making progress") {
    pros::testing::resetClock();
    setMotionCancelled(false);
    pros::MotorGroup left{1};
    pros::MotorGroup right{2};
    PID lateral(10.0, 0.0, 0.0);
    PID angular(0.0, 0.0, 0.0);
    int ticks = 0;
    pros::testing::setDelayHook([&](std::uint32_t) { ++ticks; });
    MotionSettings settings;
    settings.poseGetter = [] { return Pose(0.0, 0.0, 0.0); };
    settings.leftMotors = &left;
    settings.rightMotors = &right;
    settings.lateralPid = &lateral;
    settings.angularPid = &angular;
    settings.stallExitTime = 80_ms;
    settings.stallGraceMs = 20.0;
    moveToPoint(Pose(0.0, 48.0, 0.0), 3000_ms, {}, settings);
    pros::testing::setDelayHook(nullptr);
    CHECK(ticks < 200);
}

TEST_CASE("swingToHeading holds the named side still and drives the other") {
    // SwingSide names the side that stays put. Driving both sides is a point
    // turn, and driving the named side would rotate about the wrong corner.
    struct Case {
        SwingSide side;
        bool leftHeld;
    };
    const Case cases[] = {{SwingSide::Left, true}, {SwingSide::Right, false}};
    for (const Case& testCase : cases) {
        ClosedLoop rig(0.0, 8.0);
        rig.left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rig.right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        rig.sim.setPose(0.0, 0.0, 30.0);
        TurnToParams params;
        params.earlyExitRange = 0_deg;
        std::int32_t heldPeak = 0;
        std::int32_t drivenPeak = 0;
        pros::testing::setDelayHook([&](std::uint32_t milliseconds) {
            rig.sim.step(milliseconds);
            heldPeak = std::max(heldPeak, std::abs((testCase.leftHeld ? rig.left : rig.right).commandedVoltage()));
            drivenPeak = std::max(drivenPeak, std::abs((testCase.leftHeld ? rig.right : rig.left).commandedVoltage()));
        });
        swingToHeading(90_deg, testCase.side, 1200_ms, params, rig.settings);
        pros::testing::setDelayHook(nullptr);
        CAPTURE(testCase.leftHeld);
        CHECK(heldPeak == 0);
        CHECK(drivenPeak > 1000);
        // The brake mode must be restored to whatever the caller had set.
        CHECK((testCase.leftHeld ? rig.left : rig.right).brakeMode() == pros::E_MOTOR_BRAKE_COAST);
    }
}
