#pragma once

#include "autumn/motion/motion_common.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

namespace autumn {

struct MoveToPointParams {
    bool reversed = false;
    Number maxLateralSpeed = Number(1.0);
    // Fraction of maxVoltage. 0.8 means 80% of full scale.
    Number minLateralSpeed = Number(0.0);
    Number maxAngularSpeed = Number(1.0);
    Length earlyExitRange = 0_in;
};

void moveToPoint(Pose target, Time timeout, MoveToPointParams params, MotionSettings settings);

} // namespace autumn
