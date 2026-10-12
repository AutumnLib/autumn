#pragma once

#include "pros/error.h"

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace pros {

enum motor_brake_mode_e_t {
    E_MOTOR_BRAKE_COAST = 0,
    E_MOTOR_BRAKE_BRAKE = 1,
    E_MOTOR_BRAKE_HOLD = 2,
    E_MOTOR_BRAKE_INVALID = INT32_MAX
};

// Host stand-in for pros::MotorGroup. Positions are degrees. Voltages are millivolts.
class MotorGroup {
public:
    MotorGroup() = default;

    explicit MotorGroup(int port) : reversed_(port < 0) {}

    MotorGroup(std::initializer_list<int> ports) {
        for (const int port : ports) {
            if (port < 0) {
                reversed_ = true;
            }
        }
    }

    std::int32_t move_voltage(std::int32_t millivolts) {
        commandedMillivolts_ = millivolts;
        return 1;
    }

    std::int32_t move_velocity(std::int32_t rpm) {
        commandedRpm_ = rpm;
        return 1;
    }

    std::int32_t brake() {
        commandedMillivolts_ = 0;
        return 1;
    }

    std::int32_t set_brake_mode(motor_brake_mode_e_t mode) {
        brakeMode_ = mode;
        return 1;
    }

    std::int32_t get_current_limit() const { return connected_ ? currentLimitMa_ : PROS_ERR; }

    std::int32_t set_current_limit_all(std::int32_t milliamps) {
        currentLimitMa_ = milliamps;
        return 1;
    }

    std::vector<double> get_temperature_all() const { return temperaturesC_; }

    double get_position() const { return connected_ ? positionDegrees_ : PROS_ERR_F; }

    std::int32_t tare_position_all() {
        positionDegrees_ = 0.0;
        return 1;
    }

    double get_actual_velocity() const { return actualVelocityRpm_; }

    std::int32_t commandedVoltage() const { return commandedMillivolts_; }
    motor_brake_mode_e_t brakeMode() const { return brakeMode_; }
    bool reversed() const { return reversed_; }

    void setPositionDegrees(double degrees) { positionDegrees_ = degrees; }
    void setActualVelocityRpm(double rpm) { actualVelocityRpm_ = rpm; }
    void setConnected(bool connected) { connected_ = connected; }
    void setTemperatures(std::vector<double> celsius) { temperaturesC_ = std::move(celsius); }

private:
    bool reversed_ = false;
    bool connected_ = true;
    std::int32_t commandedMillivolts_ = 0;
    std::int32_t commandedRpm_ = 0;
    std::int32_t currentLimitMa_ = 2500;
    motor_brake_mode_e_t brakeMode_ = E_MOTOR_BRAKE_COAST;
    double positionDegrees_ = 0.0;
    double actualVelocityRpm_ = 0.0;
    std::vector<double> temperaturesC_{30.0};
};

} // namespace pros
