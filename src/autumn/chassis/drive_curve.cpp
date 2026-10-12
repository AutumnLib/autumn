#include "autumn/chassis/drive_curve.hpp"

#include <algorithm>
#include <cmath>

namespace autumn {

double ExpoDriveCurve::curve(double input) const {
    const double magnitude = std::abs(input);
    if (magnitude <= deadband_) {
        return 0.0;
    }

    const double span = 127.0 - static_cast<double>(deadband_);
    if (span <= 1e-6) {
        return 0.0;
    }
    const double x = (magnitude - static_cast<double>(deadband_)) / span;
    const double ceiling = 127.0 - static_cast<double>(minOutput_);
    double output = 0.0;
    if (std::abs(curve_ - 1.0) < 1e-6) {
        output = x * ceiling + static_cast<double>(minOutput_);
    } else {
        output = (std::pow(curve_, x) - 1.0) / (curve_ - 1.0) * ceiling + static_cast<double>(minOutput_);
    }
    return std::copysign(output, input);
}

} // namespace autumn
