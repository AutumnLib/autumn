#include "autumn/logging/logger.hpp"

#include <algorithm>
#include <cstdio>
#include <vector>

namespace autumn::logger {

namespace {
// Registry of live sinks. Sinks add/remove themselves on construct/destruct.
std::vector<Sink*>& registry() {
    static std::vector<Sink*> sinks;
    return sinks;
}

std::string formatVa(const char* format, std::va_list args) {
    std::va_list copy;
    va_copy(copy, args);
    const int needed = std::vsnprintf(nullptr, 0, format, copy);
    va_end(copy);
    if (needed <= 0) {
        return std::string{};
    }
    std::string out(static_cast<std::size_t>(needed) + 1, '\0');
    std::vsnprintf(out.data(), out.size(), format, args);
    out.pop_back(); // drop trailing NUL
    return out;
}
} // namespace

const char* levelName(Level level) {
    switch (level) {
        case Level::Debug: return "DEBUG";
        case Level::Info: return "INFO";
        case Level::Warn: return "WARN";
        case Level::Error: return "ERROR";
    }
    return "?";
}

void log(Level level, const std::string& topic, const std::string& message) {
    auto& sinks = registry();
    for (auto it = sinks.begin(); it != sinks.end();) {
        const SinkStatus status = (*it)->send(level, topic, message);
        if (status == SinkStatus::Error) {
            it = sinks.erase(it);
        } else {
            ++it;
        }
    }
}

Sink::Sink(std::string name) : name_(std::move(name)) { registry().push_back(this); }

Sink::~Sink() {
    auto& sinks = registry();
    sinks.erase(std::remove(sinks.begin(), sinks.end(), this), sinks.end());
}

void Sink::addToAllowList(const std::string& topic) { allowList_.push_back(topic); }
void Sink::removeFromAllowList(const std::string& topic) { allowList_.remove(topic); }
void Sink::addToBlockedList(const std::string& topic) { blockedList_.push_back(topic); }
void Sink::removeFromBlockedList(const std::string& topic) { blockedList_.remove(topic); }
void Sink::setLoggingLevel(Level level) { minLevel_ = level; }

SinkStatus Sink::send(Level level, const std::string& topic, const std::string& message) {
    if (static_cast<int>(level) < static_cast<int>(minLevel_)) {
        return SinkStatus::Ok;
    }
    if (!allowList_.empty() &&
        std::find(allowList_.begin(), allowList_.end(), topic) == allowList_.end()) {
        return SinkStatus::Ok;
    }
    if (std::find(blockedList_.begin(), blockedList_.end(), topic) != blockedList_.end()) {
        return SinkStatus::Ok;
    }
    return write(level, topic, message);
}

void Helper::log(Level level, const char* format, ...) const {
    std::va_list args;
    va_start(args, format);
    const std::string message = formatVa(format, args);
    va_end(args);
    logger::log(level, topic_, message);
}

void Helper::debug(const char* format, ...) const {
    std::va_list args;
    va_start(args, format);
    const std::string message = formatVa(format, args);
    va_end(args);
    logger::log(Level::Debug, topic_, message);
}

void Helper::info(const char* format, ...) const {
    std::va_list args;
    va_start(args, format);
    const std::string message = formatVa(format, args);
    va_end(args);
    logger::log(Level::Info, topic_, message);
}

void Helper::warn(const char* format, ...) const {
    std::va_list args;
    va_start(args, format);
    const std::string message = formatVa(format, args);
    va_end(args);
    logger::log(Level::Warn, topic_, message);
}

void Helper::error(const char* format, ...) const {
    std::va_list args;
    va_start(args, format);
    const std::string message = formatVa(format, args);
    va_end(args);
    logger::log(Level::Error, topic_, message);
}

} // namespace autumn::logger
