#pragma once

#include "autumn/localization/tracking_wheel.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

namespace autumn {

struct OdomConfig {
    Length trackWidth = 10.5_in;
    Length wheelDiameter = 4_in;
    TrackingWheel* verticalWheel = nullptr;
    TrackingWheel* horizontalWheel = nullptr;
    pros::Imu* imu = nullptr;
    pros::MotorGroup* leftMotors = nullptr;
    pros::MotorGroup* rightMotors = nullptr;
    bool useImeFallback = true;
    Time updatePeriod = 10_ms;
};

class Odometry {
public:
    explicit Odometry(const OdomConfig& config);

    void start();
    void stop();
    void reset(const Pose& pose = Pose{});
    void calibrateImu();

    Pose getPose() const;
    void setPose(const Pose& pose);

private:
    void updateLoop();
    void integrate(Length deltaLeft, Length deltaRight, Length deltaVertical, Angle deltaHeading);

    OdomConfig config_;
    Pose pose_;
    mutable pros::Mutex mutex_;
    pros::Task* task_ = nullptr;
    bool running_ = false;

    double prevLeft_ = 0.0;
    double prevRight_ = 0.0;
    bool hasPrevDrive_ = false;
};

} // namespace autumn
