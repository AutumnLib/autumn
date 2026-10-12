#include "autumn/localization/odometry.hpp"

#include "autumn/logging/error_codes.hpp"

#include "pros/error.h"

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
    // PROS returns 1 on success and PROS_ERR on failure.
    if (imu_->calibrate(true) == PROS_ERR) {
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
    // deltaVertical is forward (local Y). Horizontal strafe is passed as the
    // average of the drive wheels only when it was stored in deltaLeft/Right
    // by update(); the dedicated strafe distance is deltaLeft when deltaRight
    // is zero and a horizontal reading was folded into deltaLeft by update().
    // Callers pack forward in deltaVertical and strafe in deltaLeft, and pass
    // deltaRight as 0. IME forward is already folded into deltaVertical.
    const double strafe = to_inches(deltaLeft);
    const double forward = to_inches(deltaVertical);
    (void)deltaRight;
    const double dTheta = to_radians(deltaHeading);

    double localX = 0.0;
    double localY = 0.0;
    if (std::abs(dTheta) < 1e-6) {
        localX = strafe;
        localY = forward;
    } else {
        const double factor = 2.0 * std::sin(dTheta / 2.0) / dTheta;
        localX = factor * strafe;
        localY = factor * forward;
    }

    const double heading = to_radians(pose_.theta) + dTheta / 2.0;
    const double cosH = std::cos(heading);
    const double sinH = std::sin(heading);

    // LemLib 0.5.6: heading 0 = +y, clockwise positive.
    pose_.x += inches(localY * sinH - localX * cosH);
    pose_.y += inches(localY * cosH + localX * sinH);
    pose_.theta = wrapAngle(pose_.theta + deltaHeading);
}

void Odometry::update() {
    mutex_.take();
    const Angle prevTheta = pose_.theta;
    mutex_.give();

    const bool hasImu = imu_ != nullptr;
    Angle imuTheta = prevTheta;
    Angle deltaHeading{0_rad};
    if (hasImu) {
        imuTheta = imu_->getRotation();
        deltaHeading = imuTheta - prevTheta;
    }

    Length deltaLeft{0_in};
    Length deltaRight{0_in};
    const bool haveDrive = leftDrive_ != nullptr && rightDrive_ != nullptr;
    if (haveDrive) {
        const double left = to_radians(leftDrive_->getAngle()) / (2.0 * M_PI);
        const double right = to_radians(rightDrive_->getAngle()) / (2.0 * M_PI);
        const double wheelCirc =
            M_PI * to_inches(config_.wheelDiameter > 0_in ? config_.wheelDiameter : inches(4.0));
        if (hasPrevDrive_) {
            deltaLeft = inches((left - prevLeft_) * wheelCirc);
            deltaRight = inches((right - prevRight_) * wheelCirc);
        }
        prevLeft_ = left;
        prevRight_ = right;
        hasPrevDrive_ = true;
    }

    if (!hasImu && haveDrive && config_.verticalWheel == nullptr) {
        const double track = to_inches(config_.trackWidth);
        if (track > 1e-6) {
            // Left side forward is clockwise.
            deltaHeading = radians((to_inches(deltaLeft) - to_inches(deltaRight)) / track);
        }
    }

    Length forward{0_in};
    if (config_.verticalWheel != nullptr) {
        forward = config_.verticalWheel->readAndResetDelta(deltaHeading);
    } else if (haveDrive && hasPrevDrive_) {
        forward = inches((to_inches(deltaLeft) + to_inches(deltaRight)) / 2.0);
    }

    Length strafe{0_in};
    if (config_.horizontalWheel != nullptr) {
        strafe = config_.horizontalWheel->readAndResetDelta(deltaHeading);
    }

    mutex_.take();
    integrate(strafe, 0_in, forward, deltaHeading);
    if (hasImu) {
        pose_.theta = imuTheta;
    }
    mutex_.give();
}

void Odometry::updateLoop() {
    std::uint32_t prev = pros::millis();
    while (running_) {
        update();
        pros::Task::delay_until(&prev, static_cast<std::uint32_t>(to_milliseconds(config_.updatePeriod)));
    }
}

} // namespace autumn
