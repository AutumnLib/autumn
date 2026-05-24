#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace autumn {

namespace detail {

struct LengthTag {};
struct AngleTag {};
struct TimeTag {};
struct VelocityTag {};
struct AngularVelocityTag {};
struct NumberTag {};

} // namespace detail

template <typename Tag>
class Quantity {
    double value_;

public:
    constexpr explicit Quantity(double value = 0.0) : value_(value) {}

    constexpr double value() const { return value_; }

    constexpr Quantity operator+(Quantity other) const { return Quantity(value_ + other.value_); }
    constexpr Quantity operator-(Quantity other) const { return Quantity(value_ - other.value_); }
    constexpr Quantity operator*(double scalar) const { return Quantity(value_ * scalar); }
    constexpr Quantity operator/(double scalar) const { return Quantity(value_ / scalar); }
    constexpr Quantity operator-() const { return Quantity(-value_); }

    Quantity& operator+=(Quantity other) {
        value_ += other.value_;
        return *this;
    }

    Quantity& operator-=(Quantity other) {
        value_ -= other.value_;
        return *this;
    }

    Quantity& operator*=(double scalar) {
        value_ *= scalar;
        return *this;
    }

    Quantity& operator/=(double scalar) {
        value_ /= scalar;
        return *this;
    }

    constexpr bool operator==(Quantity other) const { return value_ == other.value_; }
    constexpr bool operator!=(Quantity other) const { return value_ != other.value_; }
    constexpr bool operator<(Quantity other) const { return value_ < other.value_; }
    constexpr bool operator<=(Quantity other) const { return value_ <= other.value_; }
    constexpr bool operator>(Quantity other) const { return value_ > other.value_; }
    constexpr bool operator>=(Quantity other) const { return value_ >= other.value_; }
};

template <typename Tag>
constexpr Quantity<Tag> operator*(double scalar, Quantity<Tag> quantity) {
    return quantity * scalar;
}

using Length = Quantity<detail::LengthTag>;              // inches
using Angle = Quantity<detail::AngleTag>;                // radians
using Time = Quantity<detail::TimeTag>;                  // milliseconds
using Velocity = Quantity<detail::VelocityTag>;          // inches per second
using AngularVelocity = Quantity<detail::AngularVelocityTag>; // radians per second
using Number = Quantity<detail::NumberTag>;              // dimensionless

constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kSecToMs = 1000.0;
constexpr double kMsToSec = 1.0 / 1000.0;

constexpr double to_inches(Length length) { return length.value(); }
constexpr double to_radians(Angle angle) { return angle.value(); }
constexpr double to_degrees(Angle angle) { return angle.value() * kRadToDeg; }
constexpr double to_milliseconds(Time time) { return time.value(); }
constexpr double to_seconds(Time time) { return time.value() * kMsToSec; }

constexpr Length inches(double value) { return Length(value); }
constexpr Angle radians(double value) { return Angle(value); }
constexpr Angle degrees(double value) { return Angle(value * kDegToRad); }
constexpr Time milliseconds(std::uint32_t value) { return Time(static_cast<double>(value)); }
constexpr Time seconds(double value) { return Time(value * kSecToMs); }

constexpr Velocity ips(double value) { return Velocity(value); }
constexpr AngularVelocity radps(double value) { return AngularVelocity(value); }

constexpr Length operator"" _in(long double value) { return Length(static_cast<double>(value)); }
constexpr Length operator"" _in(unsigned long long value) { return Length(static_cast<double>(value)); }

constexpr Angle operator"" _deg(long double value) {
    return Angle(static_cast<double>(value) * kDegToRad);
}

constexpr Angle operator"" _deg(unsigned long long value) {
    return Angle(static_cast<double>(value) * kDegToRad);
}

constexpr Angle operator"" _rad(long double value) { return Angle(static_cast<double>(value)); }
constexpr Angle operator"" _rad(unsigned long long value) { return Angle(static_cast<double>(value)); }

constexpr Time operator"" _ms(long double value) { return Time(static_cast<double>(value)); }
constexpr Time operator"" _ms(unsigned long long value) { return Time(static_cast<double>(value)); }

constexpr Time operator"" _sec(long double value) {
    return Time(static_cast<double>(value) * kSecToMs);
}

constexpr Time operator"" _sec(unsigned long long value) {
    return Time(static_cast<double>(value) * kSecToMs);
}

constexpr Length abs(Length value) { return Length(std::abs(value.value())); }
constexpr Angle abs(Angle value) { return Angle(std::abs(value.value())); }

inline Angle wrapAngle(Angle angle) {
    double wrapped = std::fmod(angle.value() + M_PI, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return Angle(wrapped - M_PI);
}

inline Angle shortestAngleError(Angle target, Angle current) {
    return wrapAngle(target - current);
}

constexpr Velocity operator/(Length distance, Time duration) {
    const double seconds = to_seconds(duration);
    if (seconds == 0.0) {
        return Velocity(0.0);
    }
    return Velocity(to_inches(distance) / seconds);
}

constexpr Length operator*(Velocity velocity, Time duration) {
    return Length(velocity.value() * to_seconds(duration));
}

} // namespace autumn
