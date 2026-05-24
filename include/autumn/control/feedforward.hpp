#pragma once

#include "autumn/util/units.hpp"

#include <cmath>

namespace autumn {

class Feedforward {
public:
    Feedforward() = default;

    Feedforward(double kV, double kA, double kS = 0.0) : kV_(kV), kA_(kA), kS_(kS) {}

    double calculate(double velocity, double acceleration) const {
        const double staticComponent = (std::abs(velocity) > 1e-6) ? (kS_ * (velocity > 0.0 ? 1.0 : -1.0)) : 0.0;
        return staticComponent + (kV_ * velocity) + (kA_ * acceleration);
    }

    double calculate(Velocity velocity, double acceleration) const {
        return calculate(velocity.value(), acceleration);
    }

    void setGains(double kV, double kA, double kS = 0.0) {
        kV_ = kV;
        kA_ = kA;
        kS_ = kS;
    }

    double kV() const { return kV_; }
    double kA() const { return kA_; }
    double kS() const { return kS_; }

private:
    double kV_ = 0.0;
    double kA_ = 0.0;
    double kS_ = 0.0;
};

class SimpleFeedforward {
public:
    SimpleFeedforward() = default;

    explicit SimpleFeedforward(double kV) : kV_(kV) {}

    double calculate(double velocity) const { return kV_ * velocity; }

    double calculate(Velocity velocity) const { return calculate(velocity.value()); }

    void setKV(double kV) { kV_ = kV; }

    double kV() const { return kV_; }

private:
    double kV_ = 0.0;
};

} // namespace autumn
