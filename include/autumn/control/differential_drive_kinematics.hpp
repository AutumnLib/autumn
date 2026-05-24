#pragma once

#include "autumn/control/chassis_speeds.hpp"

namespace autumn {

class DifferentialDriveKinematics {
public:
    explicit DifferentialDriveKinematics(double trackWidthInches) : trackWidthInches_(trackWidthInches) {}

    WheelSpeeds toWheelSpeeds(const ChassisSpeeds& chassis) const {
        const double halfTrack = trackWidthInches_ / 2.0;
        return WheelSpeeds{chassis.vx - chassis.omega * halfTrack, chassis.vx + chassis.omega * halfTrack};
    }

private:
    double trackWidthInches_;
};

} // namespace autumn
