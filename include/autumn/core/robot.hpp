#pragma once

#include "autumn/chassis/chassis.hpp"
#include "autumn/core/abstract_autonomous.hpp"
#include "autumn/core/abstract_config.hpp"
#include "autumn/core/abstract_driver.hpp"
#include "autumn/core/context.hpp"
#include "autumn/localization/odometry.hpp"
#include "autumn/ui/auton_selector.hpp"
#include "autumn/util/units.hpp"

#include "pros/rtos.hpp"

#include <cstdio>
#include <memory>
#include <type_traits>
#include <vector>

namespace autumn {

template <typename Config, typename Autonomous, typename Driver>
class Autumn {
public:
    static_assert(std::is_base_of_v<AbstractConfig, Config>, "Config must extend autumn::AbstractConfig");
    static_assert(std::is_base_of_v<AbstractAutonomous<Config>, Autonomous>,
                  "Autonomous must extend autumn::AbstractAutonomous<Config>");
    static_assert(std::is_base_of_v<AbstractDriver<Config>, Driver>, "Driver must extend autumn::AbstractDriver<Config>");

    Autumn(Config& config, AutumnContext<Config>& context, Autonomous& autonomous, Driver& driver,
           Chassis<Config>& chassis, Odometry& odometry, AutonSelector& autonSelector)
        : config_(config),
          context_(context),
          autonomous_(autonomous),
          driver_(driver),
          chassis_(chassis),
          odometry_(odometry),
          autonSelector_(autonSelector) {}

    void initialize() {
        context_.setChassis(chassis_);
        odometry_.calibrateImu();
        odometry_.start();
        registerAutonRoutes();
        autonSelector_.showLvglMenu();
        startPoseDisplayTask();
        onInitialize();
    }

    void competitionInitialize() { onCompetitionInitialize(); }

    void disabled() { onDisabled(); }

    void opcontrol() {
        driver_.onStart();

        // Spawn each registered driver function on its own task. Handles are
        // kept alive in driverTasks_ so the tasks persist for the whole teleop.
        for (const auto& task : driver_.tasks()) {
            driverTasks_.push_back(std::make_unique<pros::Task>(task));
        }

        // Blocks for single-loop drivers; pure multi-task drivers should leave
        // run() as the default (delay while tasks exist / teleop active).
        driver_.run();

        for (auto& task : driverTasks_) {
            task->remove();
        }
        driverTasks_.clear();

        driver_.onEnd();
    }

    AutumnContext<Config>& context() { return context_; }
    const AutumnContext<Config>& context() const { return context_; }

    Config& config() { return config_; }
    const Config& config() const { return config_; }

    Autonomous& getAutonomous() { return autonomous_; }
    const Autonomous& getAutonomous() const { return autonomous_; }

    Driver& getDriver() { return driver_; }
    const Driver& getDriver() const { return driver_; }

protected:
    virtual void onInitialize() {}
    virtual void onCompetitionInitialize() {}
    virtual void onDisabled() {}

private:
    void registerAutonRoutes() {
        for (const auto& route : autonomous_.getRoutes().routes) {
            autonSelector_.addRoutine(route.name, route.routine);
        }
    }

    void startPoseDisplayTask() {
        poseDisplayTask_ = std::make_unique<pros::Task>([this] {
            char buffer[64];
            while (true) {
                const auto pose = chassis_.getPose();
                std::snprintf(
                    buffer,
                    sizeof(buffer),
                    "X: %.2f Y: %.2f Theta: %.1f",
                    autumn::to_inches(pose.x),
                    autumn::to_inches(pose.y),
                    autumn::to_degrees(pose.theta));
                autonSelector_.setPoseText(buffer);
                pros::delay(50);
            }
        });
    }

    Config& config_;
    AutumnContext<Config>& context_;
    Autonomous& autonomous_;
    Driver& driver_;
    Chassis<Config>& chassis_;
    Odometry& odometry_;
    AutonSelector& autonSelector_;
    std::vector<std::unique_ptr<pros::Task>> driverTasks_;
    std::unique_ptr<pros::Task> poseDisplayTask_;
};

} // namespace autumn
