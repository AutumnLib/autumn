#pragma once

#include "autumn/util/units.hpp"

#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

namespace autumn::hardware {

// Abstract rotation sensor. Reports an unbounded relative angle (radians)
// measured since the last reset. Implementations must survive micro-disconnects
// without breaking the reported angle, so odometry never sees a pose jump.
class Encoder {
public:
    virtual ~Encoder() = default;

    // Unbounded angle since last reset. On hardware error, implementations
    // return the last known good value instead of an error sentinel.
    virtual Angle getAngle() const = 0;

    // Force the reported angle to a given value (rebases internal offset).
    virtual int setAngle(Angle angle) = 0;

    virtual bool isConnected() const = 0;
};

// Encoder backed by a V5 Rotation sensor. Caches the last good reading and an
// offset so that a disconnect returns the last angle and a reconnect resumes
// continuously (no jump).
class V5RotationEncoder : public Encoder {
public:
    explicit V5RotationEncoder(pros::Rotation* rotation) : rotation_(rotation) {}

    V5RotationEncoder(const V5RotationEncoder& other)
        : rotation_(other.rotation_),
          offset_(other.offset_),
          lastAngle_(other.lastAngle_),
          connectedLastCycle_(other.connectedLastCycle_) {}

    Angle getAngle() const override;
    int setAngle(Angle angle) override;
    bool isConnected() const override;

private:
    pros::Rotation* rotation_ = nullptr;
    mutable Angle offset_{0.0};
    mutable Angle lastAngle_{0.0};
    mutable bool connectedLastCycle_ = false;
    mutable pros::Mutex mutex_;
};

} // namespace autumn::hardware
