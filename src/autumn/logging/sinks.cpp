#include "autumn/logging/sinks.hpp"

#include "pros/rtos.hpp"

namespace autumn::logger {

SinkStatus TerminalSink::write(Level level, const std::string& topic, const std::string& message) {
    std::printf("[%u][%s] %s: %s\n", pros::millis(), levelName(level), topic.c_str(), message.c_str());
    return SinkStatus::Ok;
}

} // namespace autumn::logger
