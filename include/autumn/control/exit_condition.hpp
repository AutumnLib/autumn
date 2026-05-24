#pragma once

#include "autumn/util/units.hpp"

namespace autumn {

class ExitCondition {
public:
    ExitCondition(double tolerance, Time holdTime);

    bool update(double error, Time deltaTime = 10_ms);
    bool getExit() const { return exited_; }

    void setTolerance(double tolerance);
    void setHoldTime(Time holdTime);

    void reset();

private:
    double tolerance_;
    Time holdTime_;
    Time elapsed_{0_ms};
    bool exited_ = false;
};

class TimeoutExitCondition {
public:
    explicit TimeoutExitCondition(Time timeout);

    bool update(Time deltaTime);
    bool getExit() const { return exited_; }

    void reset();

private:
    Time timeout_;
    Time elapsed_{0_ms};
    bool exited_ = false;
};

} // namespace autumn
