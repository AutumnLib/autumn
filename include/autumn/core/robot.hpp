#pragma once

#include "autumn/core/base_autonomous.hpp"
#include "autumn/core/base_config.hpp"
#include "autumn/core/base_driver.hpp"
#include "autumn/core/context.hpp"

#include <type_traits>

namespace autumn {

template <typename Config, typename Autonomous, typename Driver>
class Robot {
public:
    static_assert(std::is_base_of_v<BaseConfig, Config>, "Config must extend autumn::BaseConfig");
    static_assert(std::is_base_of_v<BaseAutonomous<Config>, Autonomous>,
                  "Autonomous must extend autumn::BaseAutonomous<Config>");
    static_assert(std::is_base_of_v<BaseDriver<Config>, Driver>, "Driver must extend autumn::BaseDriver<Config>");

    Robot(Config& config, RobotContext<Config>& context, Autonomous& autonomous, Driver& driver)
        : config_(config), context_(context), autonomous_(autonomous), driver_(driver) {}

    void initialize() { onInitialize(); }

    void competitionInitialize() { onCompetitionInitialize(); }

    void disabled() { onDisabled(); }

    void opcontrol() {
        driver_.onStart();
        driver_.run();
        driver_.onEnd();
    }

    RobotContext<Config>& context() { return context_; }
    const RobotContext<Config>& context() const { return context_; }

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
    Config& config_;
    RobotContext<Config>& context_;
    Autonomous& autonomous_;
    Driver& driver_;
};

} // namespace autumn
