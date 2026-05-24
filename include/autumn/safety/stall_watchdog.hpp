#pragma once

#include "autumn/motion/motion_common.hpp"
#include "autumn/util/units.hpp"

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <functional>

namespace autumn {

class StallWatchdog {
public:
    StallWatchdog(pros::MotorGroup& left, pros::MotorGroup& right, std::function<Pose()> poseGetter,
                  Time timeout = 500_ms);

    void start();
    void stop();
    bool tripped() const { return tripped_; }

private:
    void loop();

    pros::MotorGroup& left_;
    pros::MotorGroup& right_;
    std::function<Pose()> poseGetter_;
    Time timeout_;
    pros::Task* task_ = nullptr;
    bool running_ = false;
    bool tripped_ = false;
};

} // namespace autumn
