#include "autumn/hardware/motor_group.hpp"

#include "autumn/logging/error_codes.hpp"

#include "pros/error.h"

#include <algorithm>
#include <cmath>

namespace autumn::hardware {

int MotorGroup::move(Number percent) {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    const double clamped = std::clamp(percent.value(), -1.0, 1.0);
    return group->move_voltage(static_cast<std::int32_t>(clamped * 12000.0));
}

int MotorGroup::moveVelocity(AngularVelocity velocity) {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    // rad/s -> rpm
    const double rpm = velocity.value() * (60.0 / (2.0 * M_PI));
    return group->move_velocity(static_cast<std::int32_t>(std::lround(rpm)));
}

int MotorGroup::brake() {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    return group->brake();
}

int MotorGroup::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    return group->set_brake_mode(mode);
}

double MotorGroup::getCurrentLimitA() const {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return -1.0;
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    const std::int32_t milliamps = group->get_current_limit();
    if (milliamps == PROS_ERR) {
        return -1.0;
    }
    return static_cast<double>(milliamps) / 1000.0;
}

int MotorGroup::setCurrentLimitA(double amps) {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    return group->set_current_limit_all(static_cast<std::int32_t>(amps * 1000.0));
}

std::vector<double> MotorGroup::getTemperaturesC() const {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return {};
    }
    pros::MotorGroup* group = group_;
    mutex_.give();
    return group->get_temperature_all();
}

Angle MotorGroup::getAngle() const {
    bool reportDisconnect = false;
    Angle result{0.0};

    mutex_.take();
    if (group_ == nullptr) {
        result = lastAngle_;
        mutex_.give();
        return result;
    }

    const double deg = group_->get_position();
    if (deg == PROS_ERR_F) {
        if (connectedLastCycle_) {
            connectedLastCycle_ = false;
            reportDisconnect = true;
        } else {
            connectedLastCycle_ = false;
        }
        result = lastAngle_;
        mutex_.give();
        if (reportDisconnect) {
            reportError(ErrorCode::DeviceDisconnect, "MotorGroup");
        }
        return result;
    }

    const Angle rawAngle = radians(deg * kDegToRad);
    if (!connectedLastCycle_) {
        offset_ = rawAngle - lastAngle_;
        connectedLastCycle_ = true;
    }
    lastAngle_ = rawAngle - offset_;
    result = lastAngle_;
    mutex_.give();
    return result;
}

int MotorGroup::setAngle(Angle angle) {
    mutex_.take();
    if (group_ == nullptr) {
        mutex_.give();
        return PROS_ERR;
    }
    const double deg = group_->get_position();
    const Angle rawAngle = (deg == PROS_ERR_F) ? (lastAngle_ + offset_) : radians(deg * kDegToRad);
    offset_ = rawAngle - angle;
    lastAngle_ = angle;
    mutex_.give();
    return 0;
}

bool MotorGroup::isConnected() const {
    mutex_.take();
    const bool connected = group_ != nullptr && group_->get_position() != PROS_ERR_F;
    mutex_.give();
    return connected;
}

} // namespace autumn::hardware
