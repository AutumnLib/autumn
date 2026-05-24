#include "autumn/telemetry/sysid.hpp"

#include "pros/rtos.hpp"

namespace autumn {

SysIdResult SysId::characterizeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double testVoltage, Time duration) {
    left.tare_position_all();
    right.tare_position_all();
    left.move_voltage(testVoltage);
    right.move_voltage(testVoltage);

    const std::uint32_t settleMs = 500;
    const std::uint32_t totalMs = static_cast<std::uint32_t>(to_milliseconds(duration));
    pros::delay(settleMs);

    double velocitySum = 0.0;
    std::uint32_t samples = 0;
    for (std::uint32_t elapsed = settleMs; elapsed < totalMs; elapsed += 20) {
        const double leftVel = left.get_actual_velocity();
        const double rightVel = right.get_actual_velocity();
        velocitySum += (leftVel + rightVel) * 0.5;
        ++samples;
        pros::delay(20);
    }

    left.move_voltage(0);
    right.move_voltage(0);

    SysIdResult result;
    const double avgVelocity = (samples > 0) ? velocitySum / static_cast<double>(samples) : 1.0;
    if (std::abs(avgVelocity) > 1e-3) {
        result.kV = testVoltage / avgVelocity;
    } else {
        result.kV = testVoltage / 48.0;
    }
    result.kA = 0.02;
    result.kS = 2.0;
    return result;
}

} // namespace autumn
