#pragma once

#include "pros/error.h"

#include <cstdint>

namespace pros {

// get_position() is centidegrees, matching the V5 rotation sensor.
class Rotation {
public:
    explicit Rotation(int port) : port_(port), reversed_(port < 0) {}

    std::int32_t get_position() const {
        if (!connected_) {
            return PROS_ERR;
        }
        const std::int32_t raw = centidegrees_;
        return reversed_ ? -raw : raw;
    }

    int port() const { return port_; }
    bool reversed() const { return reversed_; }

    void setCentidegrees(std::int32_t centidegrees) { centidegrees_ = centidegrees; }
    void setConnected(bool connected) { connected_ = connected; }

private:
    int port_ = 0;
    bool reversed_ = false;
    bool connected_ = true;
    std::int32_t centidegrees_ = 0;
};

} // namespace pros
