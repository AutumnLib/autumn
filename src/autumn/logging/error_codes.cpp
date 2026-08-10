#include "autumn/logging/error_codes.hpp"

#include "autumn/logging/logger.hpp"

#include "pros/screen.hpp"

#include <cstdio>

namespace autumn {

std::string errorCodeString(ErrorCode code) {
    char buffer[8];
    std::snprintf(buffer, sizeof(buffer), "E%03d", static_cast<int>(code));
    return std::string(buffer);
}

const char* errorCodeDescription(ErrorCode code) {
    switch (code) {
        case ErrorCode::None: return "no error";
        case ErrorCode::ConfigPort: return "config / port error";
        case ErrorCode::DeviceDisconnect: return "device disconnected";
        case ErrorCode::MotionTimeout: return "motion timed out";
        case ErrorCode::OdomImuCal: return "odometry / IMU calibration";
    }
    return "unknown";
}

void reportError(ErrorCode code, const std::string& detail) {
    if (code == ErrorCode::None) {
        return;
    }
    const std::string codeStr = errorCodeString(code);

    static const logger::Helper helper("autumn/error");
    if (detail.empty()) {
        helper.error("%s: %s", codeStr.c_str(), errorCodeDescription(code));
    } else {
        helper.error("%s: %s (%s)", codeStr.c_str(), errorCodeDescription(code), detail.c_str());
    }

    // Paint the compact code where it is always visible on the small screen.
    pros::screen::set_pen(pros::Color::red);
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "%s", codeStr.c_str());
}

} // namespace autumn
