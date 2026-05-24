#pragma once

#include "autumn/util/units.hpp"

#include <cmath>

namespace autumn {

class Pose {
public:
    Length x{0_in};
    Length y{0_in};
    Angle theta{0_rad};

    constexpr Pose() = default;

    constexpr Pose(Length x, Length y, Angle theta = 0_rad) : x(x), y(y), theta(theta) {}

    /// Team-facing constructor: x/y in inches, heading in degrees.
    constexpr Pose(double xInches, double yInches, double headingDegrees = 0.0)
        : x(inches(xInches)), y(inches(yInches)), theta(degrees(headingDegrees)) {}

    Pose operator+(const Pose& other) const {
        return Pose(x + other.x, y + other.y, theta);
    }

    Pose operator-(const Pose& other) const {
        return Pose(x - other.x, y - other.y, theta);
    }

    Pose operator*(double scalar) const { return Pose(x * scalar, y * scalar, theta); }

    Pose operator/(double scalar) const { return Pose(x / scalar, y / scalar, theta); }

    double dot(const Pose& other) const {
        return to_inches(x) * to_inches(other.x) + to_inches(y) * to_inches(other.y);
    }

    Length distance(const Pose& other) const {
        const double dx = to_inches(x) - to_inches(other.x);
        const double dy = to_inches(y) - to_inches(other.y);
        return inches(std::hypot(dx, dy));
    }

    Angle angleTo(const Pose& other) const {
        const double dx = to_inches(other.x) - to_inches(x);
        const double dy = to_inches(other.y) - to_inches(y);
        return radians(std::atan2(dy, dx));
    }

    Angle headingError(Angle targetHeading) const {
        return shortestAngleError(targetHeading, theta);
    }

    Pose lerp(const Pose& other, double t) const {
        return Pose(x + (other.x - x) * t, y + (other.y - y) * t, theta);
    }

    Pose rotate(Angle angle) const {
        const double c = std::cos(to_radians(angle));
        const double s = std::sin(to_radians(angle));
        const double px = to_inches(x);
        const double py = to_inches(y);
        return Pose(inches(px * c - py * s), inches(px * s + py * c), theta);
    }

    Pose withTheta(Angle newTheta) const { return Pose(x, y, newTheta); }
};

inline Pose operator*(double scalar, const Pose& pose) { return pose * scalar; }

} // namespace autumn
