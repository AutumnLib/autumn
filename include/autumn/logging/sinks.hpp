#pragma once

#include "autumn/logging/logger.hpp"

#include <cstdio>
#include <string>

namespace autumn::logger {

// Writes log lines to stdout (PROS terminal / serial).
class TerminalSink : public Sink {
public:
    TerminalSink() : Sink("terminal") {}

protected:
    SinkStatus write(Level level, const std::string& topic, const std::string& message) override;
};

} // namespace autumn::logger
