#include "autumn/localization/tracking_wheel.hpp"

#include <cmath>

namespace autumn {

namespace {
// Non-owning shared_ptr (no-op deleter) for externally owned encoders.
std::shared_ptr<hardware::Encoder> nonOwning(hardware::Encoder* encoder) {
    return std::shared_ptr<hardware::Encoder>(encoder, [](hardware::Encoder*) {});
}
} // namespace

TrackingWheel::TrackingWheel(hardware::Encoder* encoder, Length wheelDiameter, Length offsetFromCenter)
    : encoder_(nonOwning(encoder)), wheelDiameter_(wheelDiameter), offset_(offsetFromCenter) {}

TrackingWheel::TrackingWheel(pros::Rotation* encoder, Length wheelDiameter, Length offsetFromCenter)
    : encoder_(std::make_shared<hardware::V5RotationEncoder>(encoder)),
      wheelDiameter_(wheelDiameter),
      offset_(offsetFromCenter) {}

void TrackingWheel::setEncoder(hardware::Encoder* encoder) { encoder_ = nonOwning(encoder); }

void TrackingWheel::setEncoder(pros::Rotation* encoder) {
    encoder_ = std::make_shared<hardware::V5RotationEncoder>(encoder);
}

void TrackingWheel::setWheelDiameter(Length diameter) { wheelDiameter_ = diameter; }

void TrackingWheel::setOffset(Length offset) { offset_ = offset; }

Length TrackingWheel::readDistance() const {
    if (encoder_ == nullptr) {
        return 0_in;
    }
    // rotations = angle / 2pi; distance = rotations * pi * diameter = angle * diameter / 2.
    const double radiansMeasured = to_radians(encoder_->getAngle());
    return inches(radiansMeasured * to_inches(wheelDiameter_) / 2.0);
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
    // Center travel = wheel travel + offset * clockwise heading change.
    // A wheel to the right moves backward during a clockwise turn.
    delta += to_inches(offset_) * to_radians(deltaHeading);
    return inches(delta);
}

} // namespace autumn
