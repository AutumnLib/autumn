#pragma once

#include "autumn/core/abstract_config.hpp"

#include <type_traits>

namespace autumn {

template <typename Config>
class Chassis;

template <typename Config>
class AutumnContext {
public:
    static_assert(std::is_base_of_v<AbstractConfig, Config>, "Config must extend autumn::AbstractConfig");

    explicit AutumnContext(Config& config) : config_(config) {}

    Config& config() { return config_; }
    const Config& config() const { return config_; }

    // Spring-style chassis registration. The wired Chassis lives in the
    // application layer; Context holds the reference so autonomous/driver code
    // can reach it without globals. Chassis is templated, so we keep a pointer
    // to the forward-declared type to avoid a circular include with chassis.hpp.
    void setChassis(Chassis<Config>& chassis) { chassis_ = &chassis; }
    bool hasChassis() const { return chassis_ != nullptr; }
    Chassis<Config>& chassis() { return *chassis_; }
    const Chassis<Config>& chassis() const { return *chassis_; }

    DrivetrainSpec& drivetrain() { return config_.drivetrain; }
    const DrivetrainSpec& drivetrain() const { return config_.drivetrain; }

    PIDDefaults& pidDefaults() { return config_.pidDefaults; }
    const PIDDefaults& pidDefaults() const { return config_.pidDefaults; }

private:
    Config& config_;
    Chassis<Config>* chassis_ = nullptr;
};

} // namespace autumn
