#pragma once

#include "pros/error.h"

#include <cstdint>

namespace pros::adi {

class Encoder {
public:
    Encoder(char portTop, char portBottom, bool reversed = false)
        : portTop_(portTop), portBottom_(portBottom), reversed_(reversed) {
        lastCreated() = this;
    }

    static Encoder*& lastCreated() {
        static Encoder* encoder = nullptr;
        return encoder;
    }

    std::int32_t get_value() const {
        if (!connected_) {
            return PROS_ERR;
        }
        return reversed_ ? -ticks_ : ticks_;
    }

    void setTicks(std::int32_t ticks) { ticks_ = ticks; }
    void setConnected(bool connected) { connected_ = connected; }

private:
    char portTop_ = 'A';
    char portBottom_ = 'B';
    bool reversed_ = false;
    bool connected_ = true;
    std::int32_t ticks_ = 0;
};

} // namespace pros::adi
