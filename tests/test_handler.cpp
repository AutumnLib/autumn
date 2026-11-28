#include "doctest.h"

#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/motion_handler.hpp"

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

using namespace autumn;

TEST_CASE("motion handler runs, waits for a point, and reports idle afterwards") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);

    double x = 0.0;
    motion_handler::move([&] {
        for (int i = 0; i < 30; ++i) {
            x += 1.0;
            pros::delay(10);
        }
    });
    CHECK(motion_handler::isMoving());
    motion_handler::waitUntilPoint(Pose(10.0, 0.0, 0.0), 1_in, [&] { return Pose(x, 0.0, 0.0); });
    CHECK(x >= 9.0);
    while (motion_handler::isMoving()) {
        pros::delay(10);
    }
    CHECK_FALSE(motion_handler::isMoving());
}

TEST_CASE("cancel stops the running motion") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    int ticks = 0;
    motion_handler::move([&] {
        while (!motionCancelled()) {
            ++ticks;
            pros::delay(10);
        }
    });
    CHECK(motion_handler::isMoving());
    motion_handler::cancel();
    pros::delay(40);
    CHECK_FALSE(motion_handler::isMoving());
    CHECK(ticks >= 1);
}

TEST_CASE("waitUntilTraveled tracks path length from the running motion") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    resetMotionProgress();
    double y = 0.0;
    motion_handler::move([&] {
        for (int i = 0; i < 40; ++i) {
            y += 0.75;
            sampleMotionProgress(Pose(0.0, y, 0.0));
            pros::delay(10);
        }
    });
    motion_handler::waitUntilTraveled(20_in);
    CHECK(motionPathDistance() >= 19.0);
}

TEST_CASE("waitUntilTurned tracks heading change from the running motion") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    resetMotionProgress();
    double heading = 0.0;
    motion_handler::move([&] {
        for (int i = 0; i < 30; ++i) {
            heading += 3.0;
            sampleMotionProgress(Pose(0.0, 0.0, heading));
            pros::delay(10);
        }
    });
    motion_handler::waitUntilTurned(45_deg);
    CHECK(motionTurnDegrees() >= 44.0);
}

TEST_CASE("waitUntilDistance uses distance traveled, not straight-line displacement") {
    pros::testing::resetClock();
    pros::testing::setDelayHook(nullptr);
    setMotionCancelled(false);
    double y = 0.0;
    motion_handler::move([&] {
        for (int i = 0; i < 20; ++i) {
            y += (i < 10) ? 1.0 : -1.0;
            pros::delay(10);
        }
    });
    motion_handler::waitUntilDistance(15_in, [&] { return Pose(0.0, y, 0.0); });
    CHECK(motion_handler::isMoving());
    motion_handler::cancel();
    pros::delay(40);
}
