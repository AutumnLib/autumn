#pragma once

#include "autumn/motion/motion_common.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

namespace autumn {

struct MoveToPoseParams {
    bool reversed = false;
    bool lead = true;
    Number maxLateralSpeed = Number(1.0);
    Number minLateralSpeed = Number(0.0);
    Number maxAngularSpeed = Number(1.0);
    Length earlyExitRange = 0_in;
};

void moveToPose(Pose target, Time timeout, MoveToPoseParams params, MotionSettings settings);

} // namespace autumn
