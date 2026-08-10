#include "autumn/localization/odometry.hpp"

#include "autumn/logging/error_codes.hpp"

#include <cmath>

namespace autumn {

Odometry::Odometry(const OdomConfig& config) : config_(config) {
    if (config_.imu != nullptr) {
        imu_ = std::make_unique<hardware::V5IMU>(config_.imu);
    }
    if (config_.leftMotors != nullptr) {
        leftDrive_ = std::make_unique<hardware::MotorGroup>(config_.leftMotors);
    }
    if (config_.rightMotors != nullptr) {
        rightDrive_ = std::make_unique<hardware::MotorGroup>(config_.rightMotors);
    }
}

void Odometry::start() {
    if (running_) {
        return;
    }
    running_ = true;
    task_ = new pros::Task([this] { updateLoop(); });
}

void Odometry::stop() {
    running_ = false;
    if (task_ != nullptr) {
        pros::delay(static_cast<std::uint32_t>(to_milliseconds(config_.updatePeriod)) + 5);
        delete task_;
        task_ = nullptr;
    }
}

void Odometry::reset(const Pose& pose) {
    mutex_.take();
    pose_ = pose;
    hasPrevDrive_ = false;
    mutex_.give();
}

void Odometry::calibrateImu() {
    if (imu_ == nullptr) {
        return;
    }
    if (imu_->calibrate(true) != 0) {
        reportError(ErrorCode::OdomImuCal, "IMU calibrate failed");
    }
}

Pose Odometry::getPose() const {
    mutex_.take();
    const Pose copy = pose_;
    mutex_.give();
    return copy;
}

void Odometry::setPose(const Pose& pose) {
    mutex_.take();
    pose_ = pose;
    mutex_.give();
}

void Odometry::integrate(Length deltaLeft, Length deltaRight, Length deltaVertical, Angle deltaHeading) {
    const double dL = to_inches(deltaLeft);
    const double dR = to_inches(deltaRight);
    const double dV = to_inches(deltaVertical);
    const double dTheta = to_radians(deltaHeading);

    double deltaLocalX = 0.0;
    double deltaLocalY = 0.0;

    if (std::abs(dTheta) < 1e-6) {
        deltaLocalY = dV;
    } else {
        const double sinHalf = std::sin(dTheta / 2.0);
        const double factor = 2.0 * sinHalf / dTheta;
        deltaLocalY = factor * dV;
        const double avgL = (dL + dR) / 2.0;
        deltaLocalX = factor * avgL;
    }

    const double heading = to_radians(pose_.theta) + dTheta / 2.0;
    const double cosH = std::cos(heading);
    const double sinH = std::sin(heading);

    pose_.x += inches(deltaLocalY * cosH - deltaLocalX * sinH);
    pose_.y += inches(deltaLocalY * sinH + deltaLocalX * cosH);
    pose_.theta = wrapAngle(pose_.theta + deltaHeading);
}

void Odometry::updateLoop() {
    std::uint32_t prev = pros::millis();
    while (running_) {
        Length deltaLeft{0_in};
        Length deltaRight{0_in};
        Length deltaVertical{0_in};
        Angle deltaHeading{0_rad};

        const bool hasImu = imu_ != nullptr;
        if (!hasImu && config_.horizontalWheel != nullptr) {
            const Length deltaH = config_.horizontalWheel->readAndResetDelta();
            const double track = to_inches(config_.trackWidth);
            if (track > 1e-6) {
                deltaHeading = radians(to_inches(deltaH) / track);
            }
        } else if (leftDrive_ != nullptr && rightDrive_ != nullptr && config_.verticalWheel == nullptr &&
                   config_.horizontalWheel == nullptr) {
            // IME fallback when no tracking wheels: getAngle() is in radians and
            // disconnect-resilient, so revolutions = angle / 2pi.
            const double left = to_radians(leftDrive_->getAngle()) / (2.0 * M_PI);
            const double right = to_radians(rightDrive_->getAngle()) / (2.0 * M_PI);
            const double wheelCirc =
                M_PI * to_inches(config_.wheelDiameter > 0_in ? config_.wheelDiameter : inches(4.0));
            if (hasPrevDrive_) {
                deltaLeft = inches((left - prevLeft_) * wheelCirc);
                deltaRight = inches((right - prevRight_) * wheelCirc);
                const double track = to_inches(config_.trackWidth);
                if (track > 1e-6) {
                    deltaHeading = radians((to_inches(deltaRight) - to_inches(deltaLeft)) / track);
                }
            }
            prevLeft_ = left;
            prevRight_ = right;
            hasPrevDrive_ = true;
        }

        if (config_.verticalWheel != nullptr) {
            deltaVertical = config_.verticalWheel->readAndResetDelta(deltaHeading);
        }

        mutex_.take();
        if (!hasImu) {
            integrate(deltaLeft, deltaRight, deltaVertical, deltaHeading);
        } else {
            integrate(deltaLeft, deltaRight, deltaVertical, 0_rad);
            pose_.theta = imu_->getRotation();
        }
        mutex_.give();

        pros::Task::delay_until(&prev, static_cast<std::uint32_t>(to_milliseconds(config_.updatePeriod)));
    }
}

} // namespace autumn
