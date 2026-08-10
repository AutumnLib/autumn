#include "autumn/hardware/imu.hpp"

#include "autumn/logging/error_codes.hpp"

#include "pros/error.h"
#include "pros/rtos.hpp"

namespace autumn::hardware {

int V5IMU::calibrate(bool blocking) {
    mutex_.take();
    if (imu_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    pros::Imu* imu = imu_;
    mutex_.give();

    const int result = imu->reset(blocking);
    if (blocking) {
        const std::uint32_t start = pros::millis();
        while (imu->is_calibrating()) {
            if (pros::millis() - start > 5000) {
                reportError(ErrorCode::OdomImuCal, "IMU calibrate timeout");
                return PROS_ERR;
            }
            pros::delay(10);
        }
    }
    return result;
}

bool V5IMU::isCalibrating() const {
    mutex_.take();
    const bool calibrating = imu_ != nullptr && imu_->is_calibrating();
    mutex_.give();
    return calibrating;
}

Angle V5IMU::getRotation() const {
    bool reportDisconnect = false;
    Angle result{0.0};

    mutex_.take();
    if (imu_ == nullptr) {
        result = lastRotation_;
        mutex_.give();
        return result;
    }

    const double deg = imu_->get_rotation();
    if (deg == PROS_ERR_F) {
        if (connectedLastCycle_) {
            connectedLastCycle_ = false;
            reportDisconnect = true;
        } else {
            connectedLastCycle_ = false;
        }
        result = lastRotation_;
        mutex_.give();
        if (reportDisconnect) {
            reportError(ErrorCode::DeviceDisconnect, "V5IMU");
        }
        return result;
    }

    connectedLastCycle_ = true;
    lastRotation_ = radians(deg * kDegToRad) * gyroScalar_.value();
    result = lastRotation_;
    mutex_.give();
    return result;
}

int V5IMU::setRotation(Angle rotation) {
    mutex_.take();
    lastRotation_ = rotation;
    if (imu_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    const double deg = to_degrees(rotation) / gyroScalar_.value();
    pros::Imu* imu = imu_;
    mutex_.give();
    return imu->set_rotation(deg);
}

} // namespace autumn::hardware
