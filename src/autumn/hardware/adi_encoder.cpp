#include "autumn/hardware/adi_encoder.hpp"

#include "autumn/logging/error_codes.hpp"

#include "pros/error.h"

namespace autumn::hardware {

namespace {
// ADI encoder reports ticks; VEX quadrature is 360 ticks/rev.
constexpr double kTicksToRad = (2.0 * M_PI) / 360.0;
} // namespace

Angle AdiEncoder::getAngle() const {
    bool reportDisconnect = false;
    Angle result{0.0};

    mutex_.take();
    if (encoder_ == nullptr) {
        result = lastAngle_;
        mutex_.give();
        return result;
    }

    const std::int32_t raw = encoder_->get_value();
    if (raw == PROS_ERR) {
        if (connectedLastCycle_) {
            connectedLastCycle_ = false;
            reportDisconnect = true;
        } else {
            connectedLastCycle_ = false;
        }
        result = lastAngle_;
        mutex_.give();
        if (reportDisconnect) {
            reportError(ErrorCode::DeviceDisconnect, "AdiEncoder");
        }
        return result;
    }

    const Angle rawAngle = radians(static_cast<double>(raw) * kTicksToRad);
    if (!connectedLastCycle_) {
        offset_ = rawAngle - lastAngle_;
        connectedLastCycle_ = true;
    }
    lastAngle_ = rawAngle - offset_;
    result = lastAngle_;
    mutex_.give();
    return result;
}

int AdiEncoder::setAngle(Angle angle) {
    mutex_.take();
    if (encoder_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    const std::int32_t raw = encoder_->get_value();
    const Angle rawAngle =
        (raw == PROS_ERR) ? (lastAngle_ + offset_) : radians(static_cast<double>(raw) * kTicksToRad);
    offset_ = rawAngle - angle;
    lastAngle_ = angle;
    mutex_.give();
    return 0;
}

bool AdiEncoder::isConnected() const {
    mutex_.take();
    const bool connected = encoder_ != nullptr && encoder_->get_value() != PROS_ERR;
    mutex_.give();
    return connected;
}

} // namespace autumn::hardware
