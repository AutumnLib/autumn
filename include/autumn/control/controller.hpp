#pragma once

#include "autumn/control/feedforward.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/util/filters.hpp"

#include <vector>

namespace autumn {

// Composable controller interface (LemLib abstract-controllers pattern).
template <typename In, typename Out>
class Controller {
public:
    virtual ~Controller() = default;
    virtual Out calculate(const In& input) = 0;
    virtual void reset() {}
};

// Most drive-side stages map a scalar error/setpoint to a scalar command.
using ScalarController = Controller<double, double>;

class PidController : public ScalarController {
public:
    explicit PidController(PID pid) : pid_(pid) {}
    double calculate(const double& error) override { return pid_.update(error); }
    void reset() override { pid_.reset(); }
    PID& pid() { return pid_; }

private:
    PID pid_;
};

class FeedforwardController : public ScalarController {
public:
    explicit FeedforwardController(Feedforward ff) : ff_(ff) {}
    double calculate(const double& velocity) override { return ff_.calculate(velocity, 0.0); }
    Feedforward& feedforward() { return ff_; }

private:
    Feedforward ff_;
};

class SlewController : public ScalarController {
public:
    explicit SlewController(double maxDeltaPerStep) : limiter_(maxDeltaPerStep) {}
    double calculate(const double& target) override {
        current_ = limiter_.filter(target, current_);
        return current_;
    }
    void reset() override { current_ = 0.0; }

private:
    SlewRateLimiter limiter_;
    double current_ = 0.0;
};

// Wrap another ScalarController and slew-limit its output (LemLib Slewed<>).
class SlewedController : public ScalarController {
public:
    SlewedController(ScalarController& inner, double maxDeltaPerStep)
        : inner_(&inner), limiter_(maxDeltaPerStep) {}

    double calculate(const double& input) override {
        current_ = limiter_.filter(inner_->calculate(input), current_);
        return current_;
    }

    void reset() override {
        inner_->reset();
        current_ = 0.0;
    }

private:
    ScalarController* inner_;
    SlewRateLimiter limiter_;
    double current_ = 0.0;
};

// Output of each stage feeds the next (e.g. shape -> slew). LemLib
// ChainedFusionController analogue for scalar stages.
class ChainedController : public ScalarController {
public:
    void add(ScalarController* stage) { stages_.push_back(stage); }
    double calculate(const double& input) override {
        double value = input;
        for (auto* stage : stages_) {
            value = stage->calculate(value);
        }
        return value;
    }
    void reset() override {
        for (auto* stage : stages_) {
            stage->reset();
        }
    }

private:
    std::vector<ScalarController*> stages_;
};

// Sum the outputs of several stages fed the same input (e.g. FF + PID).
// LemLib AdditiveFusionController analogue.
class AdditiveController : public ScalarController {
public:
    void add(ScalarController* term) { terms_.push_back(term); }
    double calculate(const double& input) override {
        double sum = 0.0;
        for (auto* term : terms_) {
            sum += term->calculate(input);
        }
        return sum;
    }
    void reset() override {
        for (auto* term : terms_) {
            term->reset();
        }
    }

private:
    std::vector<ScalarController*> terms_;
};

} // namespace autumn
