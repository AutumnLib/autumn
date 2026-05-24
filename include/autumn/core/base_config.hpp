#pragma once

#include "autumn/control/pid.hpp"
#include "autumn/util/units.hpp"

namespace autumn {

struct DrivetrainSpec {
    double trackWidthIn = 10.5;
    double wheelDiameterIn = 4.0;
    double gearRatio = 1.0;

    Length trackWidth() const { return inches(trackWidthIn); }
    Length wheelDiameter() const { return inches(wheelDiameterIn); }
};

struct PIDDefaults {
    PID lateral{10.0, 0.0, 3.0, 5.0, true};
    PID angular{3.0, 0.0, 20.0, 5.0, true};
};

struct BaseConfig {
    virtual ~BaseConfig() = default;

    DrivetrainSpec drivetrain{};
    PIDDefaults pidDefaults{};
    std::uint32_t loopPeriodMs = 10;

    Time loopPeriod() const { return milliseconds(loopPeriodMs); }
};

} // namespace autumn
