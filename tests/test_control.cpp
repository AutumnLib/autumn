#include "support.hpp"
#include "doctest.h"

#include "autumn/control/exit_condition.hpp"
#include "autumn/control/feedforward.hpp"
#include "autumn/control/pid.hpp"
#include "autumn/motion/profile/trapezoid_profile.hpp"
#include "autumn/util/filters.hpp"

#include "pros/rtos.hpp"

using namespace autumn;

TEST_CASE("windupRange resets the integral outside the band") {
    pros::testing::resetClock();
    PID pid(0.0, 1.0, 0.0, 5.0, false);

    pid.update(100.0, 10_ms);
    const double outside = pid.update(100.0, 10_ms);
    CHECK(pid.getIntegral() == doctest::Approx(0.0));
    CHECK(outside == doctest::Approx(0.0));

    pid.reset();
    pid.update(2.0, 10_ms);
    pid.update(2.0, 10_ms);
    CHECK(pid.getIntegral() == doctest::Approx(2.0 * 0.01));

    PID open(0.0, 1.0, 0.0, 0.0, false);
    open.update(100.0, 10_ms);
    open.update(100.0, 10_ms);
    CHECK(open.getIntegral() == doctest::Approx(100.0 * 0.01));
}

TEST_CASE("sign flip still clears the integral") {
    PID pid(0.0, 1.0, 0.0, 0.0, true);
    pid.update(4.0, 10_ms);
    pid.update(4.0, 10_ms);
    CHECK(pid.getIntegral() > 0.0);
    pid.update(-1.0, 10_ms);
    // The previous integral is dropped, then the new sample is integrated.
    CHECK(pid.getIntegral() == doctest::Approx(-0.01));
}

// LemLib 0.5 gains are per 10 ms tick. Autumn divides by dt, so the same kD is ~100x stronger.
// Matching at 10 ms means kI_autumn = kI_lemlib * 100 and kD_autumn = kD_lemlib / 100.
TEST_CASE("default derivative gain matches a LemLib per-tick kD at 10 ms" * doctest::may_fail()) {
    PID pid(0.0, 0.0, 3.0, 0.0, false);
    pid.update(0.0, 10_ms);
    const double output = pid.update(10.0, 10_ms);
    CHECK(output == autumn::test::absApprox(30.0, 1e-6));
}

TEST_CASE("slew limiter steps by at most maxDelta") {
    SlewRateLimiter limiter(5.0);
    CHECK(limiter.filter(20.0, 0.0) == doctest::Approx(5.0));
    CHECK(limiter.filter(-20.0, 0.0) == doctest::Approx(-5.0));
    CHECK(limiter.filter(3.0, 0.0) == doctest::Approx(3.0));
}

TEST_CASE("exit condition holds, then resets when the error leaves the band") {
    ExitCondition exit(1.0, 30_ms);
    CHECK_FALSE(exit.update(0.5, 10_ms));
    CHECK_FALSE(exit.update(0.5, 10_ms));
    CHECK(exit.update(0.5, 10_ms));

    exit.reset();
    CHECK_FALSE(exit.update(0.2, 20_ms));
    CHECK_FALSE(exit.update(5.0, 10_ms));
    CHECK_FALSE(exit.update(0.2, 20_ms));
    CHECK(exit.update(0.2, 10_ms));
}

TEST_CASE("feedforward keeps the static sign and the velocity and accel terms") {
    Feedforward ff(2.0, 0.5, 1.0);
    CHECK(ff.calculate(4.0, 2.0) == doctest::Approx(1.0 + 8.0 + 1.0));
    CHECK(ff.calculate(-4.0, 0.0) == doctest::Approx(-1.0 - 8.0));
    CHECK(ff.calculate(0.0, 3.0) == doctest::Approx(1.5));
}

TEST_CASE("trapezoid profile ends on the requested distance for a long move") {
    TrapezoidProfile profile(ips(10.0), 10.0);
    profile.recalculate(30_in);
    CHECK(to_inches(profile.distanceAt(profile.totalTime())) == autumn::test::absApprox(30.0, 0.05));
    CHECK(profile.velocityAt(profile.totalTime()).value() == autumn::test::absApprox(0.0, 0.05));
}

TEST_CASE("short trapezoid still covers the requested distance" * doctest::may_fail()) {
    TrapezoidProfile profile(ips(10.0), 10.0);
    profile.recalculate(2_in);
    CHECK(to_inches(profile.distanceAt(profile.totalTime())) == autumn::test::absApprox(2.0, 0.05));
}

TEST_CASE("SCurveProfile is not a jerk-limited profile" * doctest::may_fail()) {
    SCurveProfile gentle(ips(48.0), 50.0);
    SCurveProfile sharp(ips(48.0), 5000.0);
    CHECK_FALSE(gentle.velocityAt(1000_ms).value() == autumn::test::absApprox(sharp.velocityAt(1000_ms).value(), 1.0));

    const double beforeStop = gentle.velocityAt(1990_ms).value();
    const double atStop = gentle.velocityAt(2000_ms).value();
    CHECK(std::abs(beforeStop - atStop) < 10.0);
}
