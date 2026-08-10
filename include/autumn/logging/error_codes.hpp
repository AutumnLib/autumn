#pragma once

#include <string>

namespace autumn {

// Compact error codes for the brain screen (too small for full messages). The
// numeric code points the user at a category; details go to the logger.
enum class ErrorCode {
    None = 0,
    ConfigPort = 1,     // E001: bad port / config
    DeviceDisconnect = 2, // E002: device disconnected mid-run
    MotionTimeout = 3,  // E003: motion exited on timeout
    OdomImuCal = 4,     // E004: odometry / IMU calibration problem
};

// "E001" style string for a code.
std::string errorCodeString(ErrorCode code);

// Human-readable category, sent to the logger alongside the code.
const char* errorCodeDescription(ErrorCode code);

// Log the error (topic "autumn/error") and paint the code on the brain screen.
void reportError(ErrorCode code, const std::string& detail = "");

} // namespace autumn
