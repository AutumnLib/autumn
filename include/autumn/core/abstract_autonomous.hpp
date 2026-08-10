#pragma once

#include "autumn/core/abstract_config.hpp"
#include "autumn/core/auton_routes.hpp"
#include "autumn/core/context.hpp"

#include <type_traits>

namespace autumn {

template <typename Config>
class AbstractAutonomous {
public:
    static_assert(std::is_base_of_v<AbstractConfig, Config>, "Config must extend autumn::AbstractConfig");

    explicit AbstractAutonomous(AutumnContext<Config>& context) : context_(context) {}

    virtual ~AbstractAutonomous() = default;

    virtual AutonRouteList getRoutes() = 0;

    virtual void onStart() {}
    virtual void onEnd() {}

protected:
    AutumnContext<Config>& context() { return context_; }
    const AutumnContext<Config>& context() const { return context_; }

    Config& config() { return context_.config(); }
    const Config& config() const { return context_.config(); }

private:
    AutumnContext<Config>& context_;
};

} // namespace autumn
