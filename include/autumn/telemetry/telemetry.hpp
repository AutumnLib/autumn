#pragma once

#include "autumn/util/pose.hpp"

#include <string>

namespace autumn {

class Telemetry {
public:
    static Telemetry& instance();

    void info(const std::string& message);
    void warn(const std::string& message);
    void logPose(const std::string& label, const Pose& pose);

private:
    Telemetry() = default;
};

} // namespace autumn
