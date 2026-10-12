#pragma once

#include "autumn/util/units.hpp"

#include "pros/motor_group.hpp"

namespace autumn {

struct SysIdResult {
    double kV = 0.0;
    double kA = 0.0;
    double kS = 0.0;
};

class SysId {
public:
    static SysIdResult characterizeDrive(pros::MotorGroup& left, pros::MotorGroup& right, double testVoltage = 6000.0,
                                         Time duration = 2000_ms);
};

} // namespace autumn
