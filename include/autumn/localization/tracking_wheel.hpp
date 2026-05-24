#pragma once

#include "autumn/util/units.hpp"

#include "pros/rotation.hpp"

namespace autumn {

class TrackingWheel {
public:
    TrackingWheel() = default;

    TrackingWheel(pros::Rotation* encoder, Length wheelDiameter, Length offsetFromCenter);
    TrackingWheel(pros::Rotation* encoder, double wheelDiameterIn, double offsetFromCenterIn)
        : TrackingWheel(encoder, inches(wheelDiameterIn), inches(offsetFromCenterIn)) {}

    void setEncoder(pros::Rotation* encoder);
    void setWheelDiameter(Length diameter);
    void setOffset(Length offset);

    Length readDistance() const;
    Length readAndResetDelta();
    Length readAndResetDelta(Angle deltaHeading);

    Length offset() const { return offset_; }

private:
    pros::Rotation* encoder_ = nullptr;
    Length wheelDiameter_{2.75_in};
    Length offset_{0_in};
    double previousPosition_ = 0.0;
    bool hasPrevious_ = false;
};

} // namespace autumn
