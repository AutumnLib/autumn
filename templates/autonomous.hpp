#pragma once

#include "autumn/chassis/chassis.hpp"
#include "autumn/core/abstract_autonomous.hpp"
#include "autumn/core/context.hpp"
#include "autumn/util/asset.hpp"
#include "config.hpp"

extern asset example_txt;

class TemplateAuto : public autumn::AbstractAutonomous<TemplateConfig> {
public:
    explicit TemplateAuto(autumn::AutumnContext<TemplateConfig>& context, autumn::Chassis<TemplateConfig>& chassis)
        : AbstractAutonomous(context), chassis_(chassis) {}

    autumn::AutonRouteList getRoutes() override {
        autumn::AutonRouteList routes;
        routes.add("Default", [this] { defaultAuto(); });
        routes.add("Skills", [this] { skillsAuto(); });
        return routes;
    }

private:
    void defaultAuto() {
        // Right-handed (clockwise) 24" box: +X then -Y then -X then +Y.
        chassis_.setPose(0, 0, 0);

        chassis_.moveToPoint(24, 0, 4000);
        chassis_.waitUntilDone();
        chassis_.turnToHeading(-90, 2000);
        chassis_.waitUntilDone();

        chassis_.moveToPoint(24, -24, 4000);
        chassis_.waitUntilDone();
        chassis_.turnToHeading(-180, 2000);
        chassis_.waitUntilDone();

        chassis_.moveToPoint(0, -24, 4000);
        chassis_.waitUntilDone();
        chassis_.turnToHeading(-270, 2000);
        chassis_.waitUntilDone();

        chassis_.moveToPoint(0, 0, 4000);
        chassis_.waitUntilDone();
        chassis_.turnToHeading(0, 2000);
        chassis_.waitUntilDone();
    }

    void skillsAuto() {
        chassis_.setPose(0, 0, 0);
        // LTV unicycle tracking is deferred. follow() is pure pursuit.
        // Boomerang is chassis_.moveToPose, not a follow mode.
        chassis_.follow(example_txt, 15, 6000, false);
        chassis_.waitUntilDone();
    }

    autumn::Chassis<TemplateConfig>& chassis_;
};
