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

} // namespace autumn::motion_handler
