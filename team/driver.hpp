#pragma once

#include "autumn/chassis/chassis.hpp"
#include "autumn/core/base_driver.hpp"
#include "autumn/core/context.hpp"
#include "team/config.hpp"

class TeamDriver : public autumn::BaseDriver<TeamConfig> {
public:
    explicit TeamDriver(autumn::RobotContext<TeamConfig>& context, autumn::Chassis<TeamConfig>& chassis)
        : BaseDriver(context), chassis_(chassis) {}

    void run() override {
        while (true) {
            const int forward = config().controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const int turn = config().controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis_.arcade(forward, turn);
            delayLoop();
        }
    }

private:
    autumn::Chassis<TeamConfig>& chassis_;
};
