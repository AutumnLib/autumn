#include "autumn/control/pid.hpp"

#include "pros/rtos.hpp"

#include <cmath>

namespace autumn {

PID::PID(double kP, double kI, double kD, double windupRange, bool signFlipReset)
    : kP_(kP),
      kI_(kI),
      kD_(kD),
      windupRange_(std::abs(windupRange)),
      signFlipReset_(signFlipReset) {}

void PID::setGains(double kP, double kI, double kD) {
    kP_ = kP;
    kI_ = kI;
    kD_ = kD;
}

void PID::setWindupRange(double windupRange) { windupRange_ = std::abs(windupRange); }

void PID::setSignFlipReset(bool signFlipReset) { signFlipReset_ = signFlipReset; }

double PID::update(double error) {
    const std::uint32_t now = pros::millis();
    if (firstUpdate_) {
        previousTimeMs_ = now;
        return update(error, 0_ms);
    }

    const double deltaSeconds = (now - previousTimeMs_) / 1000.0;
    previousTimeMs_ = now;
    return update(error, seconds(deltaSeconds));
}

double PID::update(double error, Time deltaTime) {
    const double deltaSeconds = to_seconds(deltaTime);
    if (firstUpdate_) {
        firstUpdate_ = false;
        previousError_ = error;
        output_ = kP_ * error;
        return output_;
    }

    if (deltaSeconds <= 0.0) {
        return output_;
    }

    if (signFlipReset_ && (error * previousError_ < 0.0)) {
        integral_ = 0.0;
    }

    integral_ += error * deltaSeconds;

    if (windupRange_ > 0.0) {
        integral_ = std::clamp(integral_, -windupRange_, windupRange_);
    }

    const double derivative = (error - previousError_) / deltaSeconds;
    previousError_ = error;
    output_ = (kP_ * error) + (kI_ * integral_) + (kD_ * derivative);
    return output_;
}

void PID::reset() {
    integral_ = 0.0;
    previousError_ = 0.0;
    output_ = 0.0;
    firstUpdate_ = true;
    previousTimeMs_ = pros::millis();
}

} // namespace autumn
