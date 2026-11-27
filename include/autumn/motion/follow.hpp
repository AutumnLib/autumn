#pragma once

#include "autumn/motion/motion_common.hpp"
#include "autumn/util/asset.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include <vector>

namespace autumn {

struct PathPoint {
    Length x{0_in};
    Length y{0_in};
    Velocity speed{ips(0)};
};

// follow() is pure pursuit. Boomerang is moveToPose, not a follow mode.
struct FollowParams {
    Number maxLateralSpeed = Number(1.0);
    Number maxAngularSpeed = Number(1.0);
    Length earlyExitRange = 1_in;
};

std::vector<PathPoint> parsePathAsset(const asset& pathAsset);
void follow(const asset& pathAsset, Length lookahead, Time timeout, bool reversed, FollowParams params,
            MotionSettings settings);

} // namespace autumn
