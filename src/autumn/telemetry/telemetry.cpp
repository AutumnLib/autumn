#include "autumn/telemetry/telemetry.hpp"

#include <iostream>

namespace autumn {

Telemetry& Telemetry::instance() {
    static Telemetry telemetry;
    return telemetry;
}

void Telemetry::info(const std::string& message) { std::cout << "[autumn] " << message << std::endl; }

void Telemetry::warn(const std::string& message) { std::cout << "[autumn/warn] " << message << std::endl; }

void Telemetry::logPose(const std::string& label, const Pose& pose) {
    std::cout << "[autumn/pose] " << label << " x=" << to_inches(pose.x) << " y=" << to_inches(pose.y)
              << " theta=" << to_degrees(pose.theta) << std::endl;
}

} // namespace autumn
