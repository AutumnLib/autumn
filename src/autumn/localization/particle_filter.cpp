#include "autumn/localization/particle_filter.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

namespace autumn {

ParticleFilter::ParticleFilter(std::size_t particleCount) : particleCount_(particleCount) {
    particles_.resize(particleCount_);
}

void ParticleFilter::reset(const Pose& initialPose) {
    for (auto& particle : particles_) {
        particle.pose = initialPose;
        particle.weight = 1.0 / static_cast<double>(particleCount_);
    }
    lastOdomPose_ = initialPose;
    hasLastOdomPose_ = true;
}

void ParticleFilter::predict(double linearNoise, double angularNoise) {
    static std::mt19937 rng(42);
    std::normal_distribution<double> linearDist(0.0, linearNoise);
    std::normal_distribution<double> angularDist(0.0, angularNoise);

    double deltaX = 0.0;
    double deltaY = 0.0;
    double deltaTheta = 0.0;
    if (poseGetter_) {
        const Pose current = poseGetter_();
        if (hasLastOdomPose_) {
            deltaX = to_inches(current.x) - to_inches(lastOdomPose_.x);
            deltaY = to_inches(current.y) - to_inches(lastOdomPose_.y);
            deltaTheta = to_radians(shortestAngleError(current.theta, lastOdomPose_.theta));
        }
        lastOdomPose_ = current;
        hasLastOdomPose_ = true;
    }

    for (auto& particle : particles_) {
        particle.pose.x += inches(deltaX + linearDist(rng));
        particle.pose.y += inches(deltaY + linearDist(rng));
        particle.pose.theta = wrapAngle(particle.pose.theta + radians(deltaTheta + angularDist(rng)));
    }
}

void ParticleFilter::updateWithDistance(Length measuredDistance, const Pose& landmark) {
    double total = 0.0;
    for (auto& particle : particles_) {
        const double expected = to_inches(particle.pose.distance(landmark));
        const double error = expected - to_inches(measuredDistance);
        particle.weight *= std::exp(-0.5 * error * error);
        total += particle.weight;
    }
    if (total <= 1e-9) {
        return;
    }
    for (auto& particle : particles_) {
        particle.weight /= total;
    }
}

void ParticleFilter::updateWithPose(const Pose& measurement, double confidence) {
    double total = 0.0;
    for (auto& particle : particles_) {
        const double error = to_inches(particle.pose.distance(measurement));
        particle.weight *= std::exp(-confidence * error);
        total += particle.weight;
    }
    if (total <= 1e-9) {
        return;
    }
    for (auto& particle : particles_) {
        particle.weight /= total;
    }
}

Pose ParticleFilter::estimate() const {
    double x = 0.0;
    double y = 0.0;
    double sinSum = 0.0;
    double cosSum = 0.0;
    for (const auto& particle : particles_) {
        x += to_inches(particle.pose.x) * particle.weight;
        y += to_inches(particle.pose.y) * particle.weight;
        sinSum += std::sin(to_radians(particle.pose.theta)) * particle.weight;
        cosSum += std::cos(to_radians(particle.pose.theta)) * particle.weight;
    }
    return Pose(inches(x), inches(y), radians(std::atan2(sinSum, cosSum)));
}

} // namespace autumn
