#pragma once

#include <cstdarg>
#include <list>
#include <string>

namespace autumn::logger {

enum class Level {
    Debug,
    Info,
    Warn,
    Error,
};

// Send a message to every registered sink (LemLog pattern).
void log(Level level, const std::string& topic, const std::string& message);

enum class SinkStatus {
    Ok,
    Warning,
    Error,
};

// Base class for log destinations (terminal, brain screen). Handles
// allow/block-list filtering and minimum level; subclasses implement write().
class Sink {
    friend void log(Level level, const std::string& topic, const std::string& message);

public:
    explicit Sink(std::string name);
    virtual ~Sink();

    void addToAllowList(const std::string& topic);
    void removeFromAllowList(const std::string& topic);
    void addToBlockedList(const std::string& topic);
    void removeFromBlockedList(const std::string& topic);
    void setLoggingLevel(Level level);
    const std::string& getName() const { return name_; }

protected:
    virtual SinkStatus write(Level level, const std::string& topic, const std::string& message) = 0;

private:
    SinkStatus send(Level level, const std::string& topic, const std::string& message);

    const std::string name_;
    Level minLevel_ = Level::Info;
    std::list<std::string> allowList_;
    std::list<std::string> blockedList_;
};

// Convenience per-topic logger with printf-style formatting (no std::format
// dependency, so it builds on any toolchain).
class Helper {
public:
    explicit Helper(std::string topic) : topic_(std::move(topic)) {}

    void log(Level level, const char* format, ...) const;
    void debug(const char* format, ...) const;
    void info(const char* format, ...) const;
    void warn(const char* format, ...) const;
    void error(const char* format, ...) const;

private:
    std::string topic_;
};

const char* levelName(Level level);

} // namespace autumn::logger
