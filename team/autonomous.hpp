#pragma once

#include "autumn/chassis/chassis.hpp"
#include "autumn/core/base_autonomous.hpp"
#include "autumn/core/context.hpp"
#include "autumn/util/asset.hpp"
#include "team/config.hpp"

extern asset example_txt;

class TeamAutonomous : public autumn::BaseAutonomous<TeamConfig> {
public:
    explicit TeamAutonomous(autumn::RobotContext<TeamConfig>& context, autumn::Chassis<TeamConfig>& chassis)
        : BaseAutonomous(context), chassis_(chassis) {}

    autumn::AutonRouteList getRoutes() override {
        autumn::AutonRouteList routes;
        routes.add("Default", [this] { defaultAuto(); });
        routes.add("Skills", [this] { skillsAuto(); });
        return routes;
    }

private:
    void defaultAuto() {
        chassis_.setPose(0, 0, 0);
        chassis_.moveToPoint(24, 0, 4000);
        chassis_.waitUntilDone();
        chassis_.turnToHeading(90, 2000);
        chassis_.waitUntilDone();
        chassis_.moveToPose(24, 24, 90, 4000);
        chassis_.waitUntilDone();
        chassis_.follow(example_txt, 15, 6000);
        chassis_.waitUntilDone();
    }

    void skillsAuto() {
        chassis_.setPose(0, 0, 0);
        autumn::FollowParams ltvParams;
        ltvParams.useLtvTracking = true;
        chassis_.follow(example_txt, 15, 6000, false, ltvParams);
        chassis_.waitUntilDone();
    }

    autumn::Chassis<TeamConfig>& chassis_;
};
