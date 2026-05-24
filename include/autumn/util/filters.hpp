#pragma once

#include "autumn/util/units.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

class ExponentialMovingAverage {
    double alpha_;
    double value_ = 0.0;
    bool initialized_ = false;

public:
    explicit ExponentialMovingAverage(double alpha) : alpha_(std::clamp(alpha, 0.0, 1.0)) {}

    static ExponentialMovingAverage fromTimeConstant(Time samplePeriod, Time timeConstant) {
        const double dt = to_milliseconds(samplePeriod);
        const double tau = std::max(to_milliseconds(timeConstant), 1.0);
        return ExponentialMovingAverage(dt / (dt + tau));
    }

    double filter(double sample) {
        if (!initialized_) {
            value_ = sample;
            initialized_ = true;
            return value_;
        }

        value_ = alpha_ * sample + (1.0 - alpha_) * value_;
        return value_;
    }

    double value() const { return value_; }

    void reset(double value = 0.0) {
        value_ = value;
        initialized_ = false;
    }
};

class LowPassFilter {
    Time samplePeriod_;
    Time timeConstant_;
    double value_ = 0.0;
    bool initialized_ = false;

public:
    LowPassFilter(Time samplePeriod, Time timeConstant)
        : samplePeriod_(samplePeriod), timeConstant_(timeConstant) {}

    double filter(double sample) {
        if (!initialized_) {
            value_ = sample;
            initialized_ = true;
            return value_;
        }

        const double dt = to_milliseconds(samplePeriod_);
        const double tau = std::max(to_milliseconds(timeConstant_), 1.0);
        const double alpha = dt / (dt + tau);
        value_ = alpha * sample + (1.0 - alpha) * value_;
        return value_;
    }

    double value() const { return value_; }

    void reset(double value = 0.0) {
        value_ = value;
        initialized_ = false;
    }
};

class SlewRateLimiter {
    double maxDelta_;

public:
    explicit SlewRateLimiter(double maxDeltaPerStep) : maxDelta_(std::abs(maxDeltaPerStep)) {}

    static SlewRateLimiter fromRate(double maxRatePerSecond, Time step) {
        return SlewRateLimiter(maxRatePerSecond * to_seconds(step));
    }

    double filter(double target, double current) {
        const double delta = target - current;
        if (delta > maxDelta_) {
            return current + maxDelta_;
        }
        if (delta < -maxDelta_) {
            return current - maxDelta_;
        }
        return target;
    }

    void setMaxDelta(double maxDeltaPerStep) { maxDelta_ = std::abs(maxDeltaPerStep); }
};

class DerivativeFilter {
    ExponentialMovingAverage ema_;

public:
    explicit DerivativeFilter(double alpha) : ema_(alpha) {}

    double filter(double derivativeSample) { return ema_.filter(derivativeSample); }

    void reset() { ema_.reset(); }
};

} // namespace autumn
