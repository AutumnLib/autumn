#include "autumn/localization/tracking_wheel.hpp"

#include <cmath>

namespace autumn {

TrackingWheel::TrackingWheel(pros::Rotation* encoder, Length wheelDiameter, Length offsetFromCenter)
    : encoder_(encoder), wheelDiameter_(wheelDiameter), offset_(offsetFromCenter) {}

void TrackingWheel::setEncoder(pros::Rotation* encoder) { encoder_ = encoder; }

void TrackingWheel::setWheelDiameter(Length diameter) { wheelDiameter_ = diameter; }

void TrackingWheel::setOffset(Length offset) { offset_ = offset; }

Length TrackingWheel::readDistance() const {
    if (encoder_ == nullptr) {
        return 0_in;
    }
    const double rotations = encoder_->get_position() / 36000.0;
    return inches(rotations * M_PI * to_inches(wheelDiameter_));
}

Length TrackingWheel::readAndResetDelta() { return readAndResetDelta(0_rad); }

Length TrackingWheel::readAndResetDelta(Angle deltaHeading) {
    const Length current = readDistance();
    if (!hasPrevious_) {
        hasPrevious_ = true;
        previousPosition_ = to_inches(current);
        return 0_in;
    }
    double delta = to_inches(current) - previousPosition_;
    previousPosition_ = to_inches(current);
    delta -= to_inches(offset_) * to_radians(deltaHeading);
    return inches(delta);
}

} // namespace autumn
