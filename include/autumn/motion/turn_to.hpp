#pragma once

#include "autumn/motion/motion_common.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

namespace autumn {

enum class TurnDirection { Auto, Clockwise, CounterClockwise };

struct TurnToParams {
    TurnDirection direction = TurnDirection::Auto;
    Number maxSpeed = Number(1.0);
    Number minSpeed = Number(0.0);
    Angle earlyExitRange = 0_deg;
};

void turnToHeading(Angle target, Time timeout, TurnToParams params, MotionSettings settings);
void turnToPoint(Pose target, Time timeout, TurnToParams params, MotionSettings settings);

} // namespace autumn
