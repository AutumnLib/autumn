#pragma once

#include <cstdarg>
#include <cstdio>
#include <string>

namespace pros {

enum class Color { red, black, white };

enum text_format_e_t { E_TEXT_SMALL, E_TEXT_MEDIUM, E_TEXT_LARGE };

namespace screen {

inline Color& pen() {
    static Color current = Color::black;
    return current;
}

inline std::string& lastText() {
    static std::string text;
    return text;
}

inline void set_pen(Color color) { pen() = color; }

inline void print(text_format_e_t, int, const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    lastText() = buffer;
}

inline void clearLastText() { lastText().clear(); }

} // namespace screen

} // namespace pros
