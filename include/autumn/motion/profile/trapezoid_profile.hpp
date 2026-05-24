#pragma once

#include "autumn/util/units.hpp"

namespace autumn {

class TrapezoidProfile {
public:
    TrapezoidProfile(Velocity maxVelocity, double maxAcceleration);

    Velocity velocityAt(Time elapsed) const;
    Length distanceAt(Time elapsed) const;
    Time totalTime() const { return totalTime_; }

    void setMaxVelocity(Velocity maxVelocity);
    void setMaxAcceleration(double maxAcceleration);
    void recalculate(Length distance);

private:
    Velocity maxVelocity_{ips(48)};
    double maxAcceleration_{24.0};
    Length distance_{0_in};
    Time accelTime_{0_ms};
    Time cruiseTime_{0_ms};
    Time totalTime_{0_ms};
};

class SCurveProfile {
public:
    SCurveProfile(Velocity maxVelocity, double maxJerk);

    Velocity velocityAt(Time elapsed) const;

private:
    Velocity maxVelocity_{ips(48)};
    double maxJerk_{120.0};
    Time totalTime_{2000_ms};
};

} // namespace autumn
