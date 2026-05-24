#pragma once

#include "autumn/util/units.hpp"

#include <cstdint>

namespace autumn {

class PID {
public:
    PID(double kP, double kI, double kD, double windupRange = 0.0, bool signFlipReset = false);

    double update(double error);
    double update(double error, Time deltaTime);

    void setGains(double kP, double kI, double kD);
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
