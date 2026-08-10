#include "main.h"

#include "autumn/autumn.hpp"
#include "templates/autonomous.hpp"
#include "templates/config.hpp"
#include "templates/driver.hpp"

namespace {

TemplateConfig templateConfig;
autumn::AutumnContext<TemplateConfig> autumnContext{templateConfig};
autumn::Odometry odometry{autumn::Odometry::configFrom(templateConfig)};

autumn::Chassis<TemplateConfig> chassis = autumn::Chassis<TemplateConfig>::Builder()
                                               .context(autumnContext)
                                               .odometry(odometry)
                                               .leftMotors(templateConfig.leftMotors)
                                               .rightMotors(templateConfig.rightMotors)
                                               .throttleCurve(templateConfig.throttleCurve)
                                               .steerCurve(templateConfig.steerCurve)
                                               .build();

TemplateAuto templateAuto{autumnContext, chassis};
TemplateDriver templateDriver{autumnContext, chassis};

autumn::AutonSelector autonSelector;

autumn::Autumn<TemplateConfig, TemplateAuto, TemplateDriver>& autumnInstance() {
    static autumn::Autumn<TemplateConfig, TemplateAuto, TemplateDriver> instance{
        templateConfig, autumnContext, templateAuto, templateDriver, chassis, odometry, autonSelector,
    };
    return instance;
}

} // namespace

void initialize() {
    // LVGL owns the brain screen. Do not call pros::lcd::initialize() here —
    // that installs LLEMU on the same active screen and fights the selector.
    // Use AutonSelector::showLlemuMenu() only if you intentionally drop LVGL.
    autumnInstance().initialize();
}

void disabled() { autumnInstance().disabled(); }

void competition_initialize() { autumnInstance().competitionInitialize(); }

void autonomous() { autonSelector.runSelected(autonSelector.selectedIndex()); }

void opcontrol() { autumnInstance().opcontrol(); }
