#pragma once

#include "autumn/util/units.hpp"

#include <cstdint>

namespace autumn {

class PID {
public:
    /// @param windupRange Error band, in the same units as the error passed to update().
    ///        While |error| is larger than this, the integral is reset to 0 (LemLib semantics).
    ///        0 disables the limit and the integral always accumulates.
    PID(double kP, double kI, double kD, double windupRange = 0.0, bool signFlipReset = false);

    double update(double error);
    double update(double error, Time deltaTime);

    void setGains(double kP, double kI, double kD);
    /// Sets the integral reset band. See the constructor for units and the 0 = disabled rule.
    void setWindupRange(double windupRange);
    void setSignFlipReset(bool signFlipReset);

    double getOutput() const { return output_; }
    double getIntegral() const { return integral_; }
    double getPreviousError() const { return previousError_; }

    void reset();

private:
    double kP_;
    double kI_;
    double kD_;
    double windupRange_;
    bool signFlipReset_;

    double integral_ = 0.0;
    double previousError_ = 0.0;
    double output_ = 0.0;
    bool firstUpdate_ = true;
    std::uint32_t previousTimeMs_ = 0;
};

} // namespace autumn
