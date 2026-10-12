#pragma once

#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace autumn::test {

// Differential-drive plant in the LemLib frame: +x right, +y forward,
// heading 0 = +y, clockwise positive. Wheel speeds come from commanded millivolts
// through a first-order lag. Sensors are written back onto the PROS stubs.
class DiffDriveSim {
public:
    double trackWidthIn = 10.5;
    double wheelDiameterIn = 4.0;
    // Wheel revolutions per motor revolution. 1 means the motor shaft is the wheel.
    double gearRatio = 1.0;
    double cartridgeRpm = 600.0;
    double tauSec = 0.05;
    // Scales steady-state wheel speed. 1 is no slip.
    double slip = 1.0;
    // Half-width of uniform sensor noise, in inches per step. 0 is off.
    double noiseIn = 0.0;

    void bindDrive(pros::MotorGroup* left, pros::MotorGroup* right) {
        left_ = left;
        right_ = right;
    }

    void bindImu(pros::Imu* imu) { imu_ = imu; }

    // lateralOffsetIn: positive is to the right of the robot center.
    void bindVertical(pros::Rotation* wheel, double diameterIn, double lateralOffsetIn) {
        vertical_ = wheel;
        verticalDiameterIn_ = diameterIn;
        verticalLateralOffsetIn_ = lateralOffsetIn;
    }

    // forwardOffsetIn: positive is in front of the robot center.
    void bindHorizontal(pros::Rotation* wheel, double diameterIn, double forwardOffsetIn) {
        horizontal_ = wheel;
        horizontalDiameterIn_ = diameterIn;
        horizontalForwardOffsetIn_ = forwardOffsetIn;
    }

    void setPose(double xIn, double yIn, double headingDeg) {
        x_ = xIn;
        y_ = yIn;
        theta_ = headingDeg * M_PI / 180.0;
        publishSensors();
    }

    double x() const { return x_; }
    double y() const { return y_; }
    double headingRadians() const { return theta_; }
    double headingDegrees() const { return theta_ * 180.0 / M_PI; }
    double verticalInches() const { return verticalTravel_; }
    double horizontalInches() const { return horizontalTravel_; }

    void step(std::uint32_t milliseconds) {
        const double dt = static_cast<double>(milliseconds) / 1000.0;
        if (dt <= 0.0) {
            return;
        }

        const double freeIps = (cartridgeRpm / 60.0) * gearRatio * M_PI * wheelDiameterIn * slip;
        const double targetLeft = voltageFraction(left_) * freeIps;
        const double targetRight = voltageFraction(right_) * freeIps;
        const double tau = std::max(tauSec, 1e-4);
        const double alpha = 1.0 - std::exp(-dt / tau);
        vLeft_ += (targetLeft - vLeft_) * alpha;
        vRight_ += (targetRight - vRight_) * alpha;

        const double v = 0.5 * (vLeft_ + vRight_);
        const double omega = (trackWidthIn > 1e-6) ? (vLeft_ - vRight_) / trackWidthIn : 0.0;
        const double dtheta = omega * dt;

        if (std::abs(omega) < 1e-8) {
            x_ += v * std::sin(theta_) * dt;
            y_ += v * std::cos(theta_) * dt;
        } else {
            const double radius = v / omega;
            x_ += radius * (std::cos(theta_) - std::cos(theta_ + dtheta));
            y_ += radius * (std::sin(theta_ + dtheta) - std::sin(theta_));
        }
        theta_ += dtheta;

        const double noisyForward = v * dt + sampleNoise();
        leftTravel_ += vLeft_ * dt;
        rightTravel_ += vRight_ * dt;
        verticalTravel_ += noisyForward - verticalLateralOffsetIn_ * dtheta;
        horizontalTravel_ += horizontalForwardOffsetIn_ * dtheta + sampleNoise();
        publishSensors();
    }

private:
    static double voltageFraction(const pros::MotorGroup* motor) {
        if (motor == nullptr) {
            return 0.0;
        }
        double fraction = static_cast<double>(motor->commandedVoltage()) / 12000.0;
        if (motor->reversed()) {
            fraction = -fraction;
        }
        return std::clamp(fraction, -1.0, 1.0);
    }

    static double inchesToMotorDegrees(double inches, double diameterIn, double gearRatio) {
        const double circumference = M_PI * diameterIn;
        if (circumference < 1e-9 || gearRatio == 0.0) {
            return 0.0;
        }
        const double wheelRevs = inches / circumference;
        return wheelRevs / gearRatio * 360.0;
    }

    static std::int32_t inchesToCentidegrees(double inches, double diameterIn) {
        const double degrees = inchesToMotorDegrees(inches, diameterIn, 1.0);
        return static_cast<std::int32_t>(std::lround(degrees * 100.0));
    }

    double sampleNoise() {
        if (noiseIn == 0.0) {
            return 0.0;
        }
        rng_ = rng_ * 1664525u + 1013904223u;
        const double unit = static_cast<double>(rng_ >> 8) / static_cast<double>(1u << 24);
        return (unit * 2.0 - 1.0) * noiseIn;
    }

    void publishSensors() {
        if (left_ != nullptr) {
            left_->setPositionDegrees(inchesToMotorDegrees(leftTravel_, wheelDiameterIn, gearRatio));
        }
        if (right_ != nullptr) {
            right_->setPositionDegrees(inchesToMotorDegrees(rightTravel_, wheelDiameterIn, gearRatio));
        }
        if (imu_ != nullptr) {
            imu_->setRotationDegrees(headingDegrees());
        }
        if (vertical_ != nullptr) {
            vertical_->setCentidegrees(inchesToCentidegrees(verticalTravel_, verticalDiameterIn_));
        }
        if (horizontal_ != nullptr) {
            horizontal_->setCentidegrees(inchesToCentidegrees(horizontalTravel_, horizontalDiameterIn_));
        }
    }

    pros::MotorGroup* left_ = nullptr;
    pros::MotorGroup* right_ = nullptr;
    pros::Imu* imu_ = nullptr;
    pros::Rotation* vertical_ = nullptr;
    pros::Rotation* horizontal_ = nullptr;
    double verticalDiameterIn_ = 2.75;
    double horizontalDiameterIn_ = 2.75;
    double verticalLateralOffsetIn_ = 0.0;
    double horizontalForwardOffsetIn_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double vLeft_ = 0.0;
    double vRight_ = 0.0;
    double leftTravel_ = 0.0;
    double rightTravel_ = 0.0;
    double verticalTravel_ = 0.0;
    double horizontalTravel_ = 0.0;
    std::uint32_t rng_ = 1;
};

} // namespace autumn::test
