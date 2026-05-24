#pragma once

#include <functional>
#include <string>
#include <vector>

namespace autumn {

struct AutonRoute {
    std::string name;
    std::function<void()> routine;
};

struct AutonRouteList {
    std::vector<AutonRoute> routes;

    void add(const std::string& name, std::function<void()> routine) {
        routes.push_back(AutonRoute{name, std::move(routine)});
    }
};

} // namespace autumn
