#pragma once

#include "autumn/util/pose.hpp"

#include <functional>
#include <vector>

namespace autumn {

struct Particle {
    Pose pose;
    double weight = 1.0;
};

// Deferred MCL. Weights are updated, but there is no resample step and this
// filter is not wired into Odometry. Do not use it as the pose source.
class ParticleFilter {
public:
    explicit ParticleFilter(std::size_t particleCount = 100);

    void reset(const Pose& initialPose);
    void predict(double linearNoise, double angularNoise);
    void updateWithDistance(Length measuredDistance, const Pose& landmark);
    void updateWithPose(const Pose& measurement, double confidence);
    Pose estimate() const;

    void setPoseGetter(std::function<Pose()> getter) { poseGetter_ = std::move(getter); }

private:
    std::size_t particleCount_;
    std::vector<Particle> particles_;
    std::function<Pose()> poseGetter_;
    Pose lastOdomPose_{};
    bool hasLastOdomPose_ = false;
};

} // namespace autumn
