#pragma once

namespace pros::competition {

inline bool& disabledFlag() {
    static bool disabled = false;
    return disabled;
}

inline bool is_disabled() { return disabledFlag(); }

} // namespace pros::competition
