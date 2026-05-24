#include "main.h"

#include "autumn/autumn.hpp"
#include "team/autonomous.hpp"
#include "team/config.hpp"
#include "team/driver.hpp"

namespace {

TeamConfig teamConfig;
autumn::RobotContext<TeamConfig> teamContext{teamConfig};

autumn::OdomConfig makeOdomConfig() {
    return autumn::OdomConfig{
        .trackWidth = teamConfig.drivetrain.trackWidth(),
        .wheelDiameter = teamConfig.drivetrain.wheelDiameter(),
        .verticalWheel = &teamConfig.verticalTracker,
        .horizontalWheel = &teamConfig.horizontalTracker,
        .imu = &teamConfig.imu,
        .leftMotors = &teamConfig.leftMotors,
        .rightMotors = &teamConfig.rightMotors,
        .useImeFallback = true,
        .updatePeriod = teamConfig.loopPeriod(),
    };
}

autumn::Odometry odometry{makeOdomConfig()};
autumn::Chassis<TeamConfig> chassis{teamContext, odometry, teamConfig.leftMotors, teamConfig.rightMotors,
                                    teamConfig.throttleCurve, teamConfig.steerCurve};
TeamAutonomous teamAutonomous{teamContext, chassis};
TeamDriver teamDriver{teamContext, chassis};
autumn::Robot<TeamConfig, TeamAutonomous, TeamDriver> robot{teamConfig, teamContext, teamAutonomous, teamDriver};

autumn::AutonSelector autonSelector;

autumn::Pose chassisPoseForWatchdog() { return chassis.getPose(); }

autumn::StallWatchdog stallWatchdog{teamConfig.leftMotors, teamConfig.rightMotors, chassisPoseForWatchdog};

} // namespace

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Autumn PROS Template");

    odometry.calibrateImu();
    odometry.start();
    stallWatchdog.start();

    for (const auto& route : teamAutonomous.getRoutes().routes) {
        autonSelector.addRoutine(route.name, [&route] {
            route.routine();
        });
    }
    autonSelector.showLlemuMenu();

    pros::Task screenTask([] {
        while (true) {
            const auto pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f Y: %.2f", autumn::to_inches(pose.x), autumn::to_inches(pose.y));
            pros::lcd::print(1, "Theta: %.1f", autumn::to_degrees(pose.theta));
            pros::delay(50);
        }
    });

    robot.initialize();
}

void disabled() { robot.disabled(); }

void competition_initialize() {
    autonSelector.showLlemuMenu();
    robot.competitionInitialize();
}

void autonomous() { autonSelector.runSelected(autonSelector.selectedIndex()); }

void opcontrol() { robot.opcontrol(); }
