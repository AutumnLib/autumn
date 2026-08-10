#pragma once

#include "autumn/hardware/encoder.hpp"

#include "pros/adi.hpp"
#include "pros/rtos.hpp"

#include <memory>

namespace autumn::hardware {

// Encoder backed by a PROS ADI quadrature encoder. Same disconnect-resilience
// pattern as V5RotationEncoder (last-good + reconnect rebase).
class AdiEncoder : public Encoder {
public:
    // Owns a pros::adi::Encoder constructed from the two ADI ports.
    AdiEncoder(char portTop, char portBottom, bool reversed = false)
        : encoder_(std::make_unique<pros::adi::Encoder>(portTop, portBottom, reversed)) {}

    AdiEncoder(const AdiEncoder& other) = delete;
    AdiEncoder& operator=(const AdiEncoder& other) = delete;

    Angle getAngle() const override;
    int setAngle(Angle angle) override;
    bool isConnected() const override;

private:
    std::unique_ptr<pros::adi::Encoder> encoder_;
    mutable Angle offset_{0.0};
    mutable Angle lastAngle_{0.0};
    mutable bool connectedLastCycle_ = false;
    mutable pros::Mutex mutex_;
};

} // namespace autumn::hardware
