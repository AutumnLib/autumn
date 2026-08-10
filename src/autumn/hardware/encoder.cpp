#include "autumn/hardware/encoder.hpp"

#include "autumn/logging/error_codes.hpp"

#include "pros/error.h"

namespace autumn::hardware {

namespace {
// pros::Rotation::get_position() returns centidegrees (1/100 deg).
constexpr double kCentidegToRad = kDegToRad / 100.0;
} // namespace

Angle V5RotationEncoder::getAngle() const {
    bool reportDisconnect = false;
    Angle result{0.0};

    mutex_.take();
    if (rotation_ == nullptr) {
        result = lastAngle_;
        mutex_.give();
        return result;
    }

    const std::int32_t raw = rotation_->get_position();
    if (raw == PROS_ERR) {
        // Disconnected: hold last good angle. Report only on the falling edge.
        if (connectedLastCycle_) {
            connectedLastCycle_ = false;
            reportDisconnect = true;
        } else {
            connectedLastCycle_ = false;
        }
        result = lastAngle_;
        mutex_.give();
        if (reportDisconnect) {
            reportError(ErrorCode::DeviceDisconnect, "V5RotationEncoder");
        }
        return result;
    }

    const Angle rawAngle = radians(static_cast<double>(raw) * kCentidegToRad);
    if (!connectedLastCycle_) {
        // Reconnected (or first read): rebase so reported angle stays continuous.
        offset_ = rawAngle - lastAngle_;
        connectedLastCycle_ = true;
    }

    lastAngle_ = rawAngle - offset_;
    result = lastAngle_;
    mutex_.give();
    return result;
}

int V5RotationEncoder::setAngle(Angle angle) {
    mutex_.take();
    if (rotation_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    const std::int32_t raw = rotation_->get_position();
    const Angle rawAngle =
        (raw == PROS_ERR) ? (lastAngle_ + offset_) : radians(static_cast<double>(raw) * kCentidegToRad);
    offset_ = rawAngle - angle;
    lastAngle_ = angle;
    mutex_.give();
    return 0;
}

bool V5RotationEncoder::isConnected() const {
    mutex_.take();
    const bool connected = rotation_ != nullptr && rotation_->get_position() != PROS_ERR;
    mutex_.give();
    return connected;
}

} // namespace autumn::hardware
