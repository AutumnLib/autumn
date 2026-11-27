#pragma once

#include "autumn/control/pid.hpp"
#include "autumn/util/units.hpp"

#include <any>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace autumn {

struct DrivetrainSpec {
    double trackWidthIn = 10.5;
    double wheelDiameterIn = 4.0;
    double gearRatio = 1.0;

    Length trackWidth() const { return inches(trackWidthIn); }
    Length wheelDiameter() const { return inches(wheelDiameterIn); }
};

struct PIDDefaults {
    // dt-based PID. LemLib 0.5 per-tick gains convert at 10 ms as kI*100 and kD/100.
    PID lateral{10.0, 0.0, 0.03, 5.0, true};
    PID angular{3.0, 0.0, 0.2, 5.0, true};
};

struct AbstractConfig {
    virtual ~AbstractConfig() = default;

    DrivetrainSpec drivetrain{};
    PIDDefaults pidDefaults{};

    // Arbitrary user-value bag (motors, pistons, sensors, scalars). Stored by
    // string key so autonomous/driver code can retrieve them via config().
    template <typename T>
    void put(const std::string& key, T value) {
        extras_[key] = std::any(std::move(value));
    }

    template <typename T>
    T& get(const std::string& key) {
        auto it = extras_.find(key);
        if (it == extras_.end()) {
            throw std::out_of_range("autumn::AbstractConfig: no value for key '" + key + "'");
        }
        return std::any_cast<T&>(it->second);
    }

    template <typename T>
    const T& get(const std::string& key) const {
        auto it = extras_.find(key);
        if (it == extras_.end()) {
            throw std::out_of_range("autumn::AbstractConfig: no value for key '" + key + "'");
        }
        return std::any_cast<const T&>(it->second);
    }

    bool has(const std::string& key) const { return extras_.find(key) != extras_.end(); }

private:
    std::unordered_map<std::string, std::any> extras_;
};

} // namespace autumn
