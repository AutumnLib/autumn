#pragma once

#include "autumn/core/base_config.hpp"

#include <type_traits>

namespace autumn {

template <typename Config>
class RobotContext {
public:
    static_assert(std::is_base_of_v<BaseConfig, Config>, "Config must extend autumn::BaseConfig");

    explicit RobotContext(Config& config) : config_(config) {}

    Config& config() { return config_; }
    const Config& config() const { return config_; }

    DrivetrainSpec& drivetrain() { return config_.drivetrain; }
    const DrivetrainSpec& drivetrain() const { return config_.drivetrain; }

    PIDDefaults& pidDefaults() { return config_.pidDefaults; }
    const PIDDefaults& pidDefaults() const { return config_.pidDefaults; }

    Time loopPeriod() const { return config_.loopPeriod(); }

private:
    Config& config_;
};

} // namespace autumn
