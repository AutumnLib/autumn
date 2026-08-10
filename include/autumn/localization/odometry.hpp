#pragma once

#include "autumn/hardware/imu.hpp"
#include "autumn/hardware/motor_group.hpp"
#include "autumn/localization/tracking_wheel.hpp"
#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <memory>

namespace autumn {

struct OdomConfig {
    Length trackWidth = 10.5_in;
    Length wheelDiameter = 4_in;
    TrackingWheel* verticalWheel = nullptr;
    TrackingWheel* horizontalWheel = nullptr;
    pros::Imu* imu = nullptr;
    pros::MotorGroup* leftMotors = nullptr;
    pros::MotorGroup* rightMotors = nullptr;
    Time updatePeriod = 10_ms;
};

class Odometry {
public:
    explicit Odometry(const OdomConfig& config);

    template <typename Config>
    static OdomConfig configFrom(Config& config) {
        return OdomConfig{
            .trackWidth = config.drivetrain.trackWidth(),
            .wheelDiameter = config.drivetrain.wheelDiameter(),
            .verticalWheel = &config.verticalTracker,
            .horizontalWheel = &config.horizontalTracker,
            .imu = &config.imu,
            .leftMotors = &config.leftMotors,
            .rightMotors = &config.rightMotors,
        };
    }

    class Builder {
    public:
        Builder& trackWidth(Length width) {
            config_.trackWidth = width;
            return *this;
        }
        Builder& wheelDiameter(Length diameter) {
            config_.wheelDiameter = diameter;
            return *this;
        }
        Builder& verticalWheel(TrackingWheel* wheel) {
            config_.verticalWheel = wheel;
            return *this;
        }
        Builder& horizontalWheel(TrackingWheel* wheel) {
            config_.horizontalWheel = wheel;
            return *this;
        }
        Builder& imu(pros::Imu* imuDevice) {
            config_.imu = imuDevice;
            return *this;
        }
        Builder& leftMotors(pros::MotorGroup* motors) {
            config_.leftMotors = motors;
            return *this;
        }
        Builder& rightMotors(pros::MotorGroup* motors) {
            config_.rightMotors = motors;
            return *this;
        }
        Builder& updatePeriod(Time period) {
            config_.updatePeriod = period;
            return *this;
        }

        template <typename Config>
        Builder& fromConfig(Config& config) {
            config_ = Odometry::configFrom(config);
            return *this;
        }

        Odometry build() const { return Odometry(config_); }

    private:
        OdomConfig config_{};
    };

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

    // HAL wrappers around the raw PROS devices in config_, giving disconnect
    // resilience for free. Built in the constructor.
    std::unique_ptr<hardware::IMU> imu_;
    std::unique_ptr<hardware::MotorGroup> leftDrive_;
    std::unique_ptr<hardware::MotorGroup> rightDrive_;

    double prevLeft_ = 0.0;
    double prevRight_ = 0.0;
    bool hasPrevDrive_ = false;
};

} // namespace autumn
