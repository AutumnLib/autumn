#pragma once

#include "autumn/util/units.hpp"

#include "pros/imu.hpp"
#include "pros/rtos.hpp"

namespace autumn::hardware {

// Abstract inertial sensor. Reports an unbounded heading (radians). A gyro
// scalar corrects a constant multiplicative drift error.
class IMU {
public:
    virtual ~IMU() = default;

    virtual int calibrate(bool blocking = true) = 0;
    virtual bool isCalibrating() const = 0;

    // Unbounded heading. On hardware error, returns the last known good value.
    virtual Angle getRotation() const = 0;
    virtual int setRotation(Angle rotation) = 0;

    void setGyroScalar(Number scalar) { gyroScalar_ = scalar; }
    Number gyroScalar() const { return gyroScalar_; }

protected:
    Number gyroScalar_{1.0};
};

// IMU backed by a V5 Inertial sensor, with disconnect resilience (holds last
// good heading) and an applied gyro scalar.
class V5IMU : public IMU {
public:
    explicit V5IMU(pros::Imu* imu, Number gyroScalar = Number{1.0}) : imu_(imu) { gyroScalar_ = gyroScalar; }

    V5IMU(const V5IMU& other)
        : imu_(other.imu_), lastRotation_(other.lastRotation_), connectedLastCycle_(other.connectedLastCycle_) {
        gyroScalar_ = other.gyroScalar_;
    }

    int calibrate(bool blocking = true) override;
    bool isCalibrating() const override;
    Angle getRotation() const override;
    int setRotation(Angle rotation) override;

private:
    pros::Imu* imu_ = nullptr;
    mutable Angle lastRotation_{0.0};
    mutable bool connectedLastCycle_ = false;
    mutable pros::Mutex mutex_;
};

} // namespace autumn::hardware
