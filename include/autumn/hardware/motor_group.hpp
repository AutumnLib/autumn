#pragma once

#include "autumn/hardware/encoder.hpp"
#include "autumn/util/units.hpp"

#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

#include <vector>

namespace autumn::hardware {

// Thin facade over a pros::MotorGroup. Unitized inputs/outputs, normalized
// power in [-1, 1], and an Encoder face (average angle) for IME-based odometry
// fallback with disconnect resilience.
class MotorGroup : public Encoder {
public:
    explicit MotorGroup(pros::MotorGroup* group) : group_(group) {}

    MotorGroup(const MotorGroup& other)
        : group_(other.group_),
          offset_(other.offset_),
          lastAngle_(other.lastAngle_),
          connectedLastCycle_(other.connectedLastCycle_) {}

    // Normalized power in [-1.0, +1.0].
    int move(Number percent);
    int moveVelocity(AngularVelocity velocity);
    int brake();
    int setBrakeMode(pros::motor_brake_mode_e_t mode);

    // Combined current limit of the group (amps). Returns -1 on failure.
    double getCurrentLimitA() const;
    int setCurrentLimitA(double amps);

    // Per-motor temperatures in Celsius. Empty vector on failure.
    std::vector<double> getTemperaturesC() const;

    // Encoder face: average angle of the group (radians, unbounded).
    Angle getAngle() const override;
    int setAngle(Angle angle) override;
    bool isConnected() const override;

private:
    pros::MotorGroup* group_ = nullptr;
    mutable Angle offset_{0.0};
    mutable Angle lastAngle_{0.0};
    mutable bool connectedLastCycle_ = false;
    mutable pros::Mutex mutex_;
};

} // namespace autumn::hardware
