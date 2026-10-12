#include "support.hpp"
#include "doctest.h"

#include "autumn/util/pose.hpp"
#include "autumn/util/units.hpp"

#include <cmath>

using namespace autumn;

TEST_CASE("unit conversions and literals") {
    CHECK(to_inches(12_in) == doctest::Approx(12.0));
    CHECK(to_inches(3.5_in) == doctest::Approx(3.5));
    CHECK(to_degrees(90_deg) == doctest::Approx(90.0));
    CHECK(to_radians(180_deg) == doctest::Approx(M_PI));
    CHECK(to_milliseconds(1500_ms) == doctest::Approx(1500.0));
    CHECK(to_seconds(2_sec) == doctest::Approx(2.0));
    CHECK(to_inches((24_in / 2_sec) * 500_ms) == doctest::Approx(6.0));
}

TEST_CASE("wrapAngle and shortest error stay in [-pi, pi]") {
    CHECK(to_degrees(wrapAngle(270_deg)) == autumn::test::absApprox(-90.0, 1e-6));
    CHECK(to_degrees(wrapAngle(-270_deg)) == autumn::test::absApprox(90.0, 1e-6));
    CHECK(to_degrees(shortestAngleError(10_deg, 350_deg)) == autumn::test::absApprox(20.0, 1e-4));
    CHECK(to_degrees(shortestAngleError(350_deg, 10_deg)) == autumn::test::absApprox(-20.0, 1e-4));
}

TEST_CASE("angleTo uses the LemLib heading: 0 is +y, clockwise positive" * doctest::may_fail()) {
    const Pose origin(0.0, 0.0, 0.0);
    // Straight ahead is +y. LemLib heading to that point is 0, not +90.
    CHECK(to_degrees(origin.angleTo(Pose(0.0, 10.0, 0.0))) == autumn::test::absApprox(0.0, 1e-4));
    // Straight right is +x. LemLib heading is +90 (clockwise from +y).
    CHECK(to_degrees(origin.angleTo(Pose(10.0, 0.0, 0.0))) == autumn::test::absApprox(90.0, 1e-4));
}
