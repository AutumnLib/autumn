#pragma once

#include "autumn/core/auton_routes.hpp"
#include "autumn/core/context.hpp"

#include <type_traits>

namespace autumn {

template <typename Config>
class BaseAutonomous {
public:
    static_assert(std::is_base_of_v<BaseConfig, Config>, "Config must extend autumn::BaseConfig");

    explicit BaseAutonomous(RobotContext<Config>& context) : context_(context) {}

    virtual ~BaseAutonomous() = default;

    virtual AutonRouteList getRoutes() = 0;

    virtual void onStart() {}
    virtual void onEnd() {}

protected:
    RobotContext<Config>& context() { return context_; }
    const RobotContext<Config>& context() const { return context_; }

    Config& config() { return context_.config(); }
    const Config& config() const { return context_.config(); }

private:
    RobotContext<Config>& context_;
};

} // namespace autumn
