#pragma once

#include "autumn/hardware/encoder.hpp"
#include "autumn/util/units.hpp"

#include "pros/rotation.hpp"

#include <memory>

namespace autumn {

class TrackingWheel {
public:
    TrackingWheel() = default;

    // Interface-based: consumes any hardware::Encoder (non-owning).
    TrackingWheel(hardware::Encoder* encoder, Length wheelDiameter, Length offsetFromCenter);

    // Adapter ctors: wrap a raw pros::Rotation in an owned V5RotationEncoder so
    // existing team configs keep compiling.
    TrackingWheel(pros::Rotation* encoder, Length wheelDiameter, Length offsetFromCenter);
    TrackingWheel(pros::Rotation* encoder, double wheelDiameterIn, double offsetFromCenterIn)
        : TrackingWheel(encoder, inches(wheelDiameterIn), inches(offsetFromCenterIn)) {}

    void setEncoder(hardware::Encoder* encoder);
    void setEncoder(pros::Rotation* encoder);
    void setWheelDiameter(Length diameter);
    void setOffset(Length offset);

    Length readDistance() const;
    Length readAndResetDelta();
    Length readAndResetDelta(Angle deltaHeading);

    Length offset() const { return offset_; }

private:
    std::shared_ptr<hardware::Encoder> encoder_;
    Length wheelDiameter_{2.75_in};
    Length offset_{0_in};
    double previousPosition_ = 0.0;
    bool hasPrevious_ = false;
};

} // namespace autumn
