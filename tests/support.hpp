#pragma once

#include "doctest.h"
#include "sim/diff_drive_sim.hpp"

#include "autumn/motion/motion_common.hpp"

#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace autumn::test {

// Absolute tolerance. doctest 2.4 Approx only has relative epsilon/scale.
inline doctest::Approx absApprox(double value, double tolerance) {
    return doctest::Approx(value).epsilon(tolerance / 1.0e6).scale(1.0e6);
}

struct PlantHook {
    explicit PlantHook(DiffDriveSim& sim) : sim_(sim) {
        pros::testing::resetClock();
        setMotionCancelled(false);
        pros::screen::clearLastText();
        pros::testing::setDelayHook([this](std::uint32_t milliseconds) { sim_.step(milliseconds); });
    }

    ~PlantHook() { pros::testing::setDelayHook(nullptr); }

    DiffDriveSim& sim_;
};

inline Pose simPose(const DiffDriveSim& sim) { return Pose(sim.x(), sim.y(), sim.headingDegrees()); }

} // namespace autumn::test
