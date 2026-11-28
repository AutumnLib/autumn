#pragma once

#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include <functional>

namespace autumn::motion_handler {

void move(std::function<void()> motion);
bool isMoving();
void cancel();
void waitUntilPoint(Pose target, Length radius, std::function<Pose()> poseGetter);
void waitUntilDistance(Length distance, std::function<Pose()> poseGetter);
// Distance and heading accumulated by the running motion (not straight-line displacement).
void waitUntilTraveled(Length distance);
void waitUntilTurned(Angle amount);

} // namespace autumn::motion_handler
