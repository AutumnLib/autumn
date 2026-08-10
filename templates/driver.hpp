#pragma once

#include "autumn/chassis/chassis.hpp"
#include "autumn/core/abstract_driver.hpp"
#include "autumn/core/context.hpp"
#include "config.hpp"

class TemplateDriver : public autumn::AbstractDriver<TemplateConfig> {
public:
    explicit TemplateDriver(autumn::AutumnContext<TemplateConfig>& context, autumn::Chassis<TemplateConfig>& chassis)
        : AbstractDriver(context), chassis_(chassis) {
        // Example secondary driver task: runs on its own pros::Task alongside
        // run(). Reads config() like the main loop. Replace the body with real
        // mechanism code (intake, lift, etc.).
        registerTask([this] {
            while (true) {
                (void)config(); // touch config so the DI path is exercised
                delayLoop();
            }
        });
    }

    void run() override {
        while (true) {
            const int forward = config().controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            const int turn = config().controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis_.arcade(forward, turn);
            delayLoop();
        }
    }

private:
    autumn::Chassis<TemplateConfig>& chassis_;
};
