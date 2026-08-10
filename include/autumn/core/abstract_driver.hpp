#pragma once

#include "autumn/core/abstract_config.hpp"
#include "autumn/core/context.hpp"

#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <cstdint>
#include <functional>
#include <type_traits>
#include <utility>
#include <vector>

namespace autumn {

template <typename Config>
class AbstractDriver {
public:
    static_assert(std::is_base_of_v<AbstractConfig, Config>, "Config must extend autumn::AbstractConfig");

    explicit AbstractDriver(AutumnContext<Config>& context) : context_(context) {}

    virtual ~AbstractDriver() = default;

    // Optional single teleop loop. Override for a blocking arcade/control loop.
    // Pure multi-task drivers can leave this default: it parks until disabled so
    // registered tasks keep running for the whole opcontrol phase.
    virtual void run() {
        while (!pros::competition::is_disabled()) {
            delayLoop();
        }
    }

    virtual void onStart() {}
    virtual void onEnd() {}

    const std::vector<std::function<void()>>& tasks() const { return tasks_; }

protected:
    // Register a function to run concurrently on its own pros::Task during
    // opcontrol. Each task can read config() like the rest of the driver.
    void registerTask(std::function<void()> task) { tasks_.push_back(std::move(task)); }

    AutumnContext<Config>& context() { return context_; }
    const AutumnContext<Config>& context() const { return context_; }

    Config& config() { return context_.config(); }
    const Config& config() const { return context_.config(); }

    // Driver-local period. Not tied to config/odom loop timing.
    void delayLoop(std::uint32_t delayMs = 10) const { pros::delay(delayMs); }

private:
    AutumnContext<Config>& context_;
    std::vector<std::function<void()>> tasks_;
};

} // namespace autumn
