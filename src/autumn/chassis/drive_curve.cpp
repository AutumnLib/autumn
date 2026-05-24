#include "autumn/chassis/drive_curve.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

double ExpoDriveCurve::curve(double input) const {
    const double sign = input >= 0 ? 1.0 : -1.0;
    const double magnitude = std::abs(input);
    if (magnitude <= deadband_) {
        return 0.0;
    }

    const double normalized = (127.0 - deadband_ - minOutput_) * std::pow((magnitude - deadband_) / (127.0 - deadband_), curve_) /
                              (127.0 - deadband_);
    return sign * (normalized + minOutput_);
}

} // namespace autumn
