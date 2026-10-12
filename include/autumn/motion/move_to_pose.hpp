#pragma once

#include "autumn/motion/motion_common.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

namespace autumn {

struct MoveToPoseParams {
    bool reversed = false;
    // How far the carrot sits back along the target heading, as a fraction of distance.
    double lead = 0.6;
    double horizontalDrift = 2.0;
    Number maxLateralSpeed = Number(1.0);
    Number minLateralSpeed = Number(0.0);
    Number maxAngularSpeed = Number(1.0);
    Length earlyExitRange = 0_in;
};

void moveToPose(Pose target, Time timeout, MoveToPoseParams params, MotionSettings settings);

} // namespace autumn
