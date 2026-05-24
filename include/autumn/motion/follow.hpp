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

enum class FollowMode { PurePursuit, Boomerang };

struct FollowParams {
    FollowMode mode = FollowMode::PurePursuit;
    bool useLtvTracking = false;
    Number maxLateralSpeed = Number(1.0);
    Number maxAngularSpeed = Number(1.0);
    Length earlyExitRange = 1_in;
    double ltvMaxXErrorIn = 2.5;
    double ltvMaxYErrorIn = 5.0;
    double ltvMaxThetaErrorRad = 2.0;
};

std::vector<PathPoint> parsePathAsset(const asset& pathAsset);
void follow(const asset& pathAsset, Length lookahead, Time timeout, bool reversed, FollowParams params,
            MotionSettings settings);

} // namespace autumn
