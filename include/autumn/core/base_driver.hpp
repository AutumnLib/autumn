#pragma once

#include "autumn/core/context.hpp"
#include "autumn/util/units.hpp"

#include "pros/rtos.hpp"

#include <cstdint>
#include <type_traits>

namespace autumn {

template <typename Config>
class BaseDriver {
public:
    static_assert(std::is_base_of_v<BaseConfig, Config>, "Config must extend autumn::BaseConfig");

    explicit BaseDriver(RobotContext<Config>& context) : context_(context) {}

    virtual ~BaseDriver() = default;

    virtual void run() = 0;

    virtual void onStart() {}
    virtual void onEnd() {}

protected:
    RobotContext<Config>& context() { return context_; }
    const RobotContext<Config>& context() const { return context_; }

    Config& config() { return context_.config(); }
    const Config& config() const { return context_.config(); }

    void delayLoop() const { pros::delay(static_cast<std::uint32_t>(to_milliseconds(context_.loopPeriod()))); }

private:
    RobotContext<Config>& context_;
};

} // namespace autumn
