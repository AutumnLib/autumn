#pragma once

#include "pros/error.h"

#include <cstdint>

namespace pros {

// get_rotation() is unbounded clockwise-positive degrees, matching the V5 inertial sensor.
class Imu {
public:
    explicit Imu(int port) : port_(port) {}

    std::int32_t reset(bool blocking = false) const {
        (void)blocking;
        resetCount_ += 1;
        return 1;
    }

    bool is_calibrating() const { return calibrating_; }

    double get_rotation() const { return connected_ ? rotationDegrees_ : PROS_ERR_F; }

    std::int32_t set_rotation(double degrees) {
        rotationDegrees_ = degrees;
        return 1;
    }

    int port() const { return port_; }
    int resetCount() const { return resetCount_; }

    void setCalibrating(bool calibrating) const { calibrating_ = calibrating; }
    void setConnected(bool connected) const { connected_ = connected; }
    void setRotationDegrees(double degrees) const { rotationDegrees_ = degrees; }

private:
    int port_ = 0;
    mutable bool calibrating_ = false;
    mutable bool connected_ = true;
    mutable double rotationDegrees_ = 0.0;
    mutable int resetCount_ = 0;
};

} // namespace pros
