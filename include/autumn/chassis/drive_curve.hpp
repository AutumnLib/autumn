#pragma once

#include "autumn/util/units.hpp"

namespace autumn {

class ExpoDriveCurve {
public:
    ExpoDriveCurve(int deadband = 3, int minOutput = 10, double curve = 1.019)
        : deadband_(deadband), minOutput_(minOutput), curve_(curve) {}

    double curve(double input) const;

private:
    int deadband_;
    int minOutput_;
    double curve_;
};

} // namespace autumn
