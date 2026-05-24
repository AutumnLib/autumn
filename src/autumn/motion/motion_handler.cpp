#include "autumn/motion/motion_common.hpp"
#include "autumn/motion/motion_handler.hpp"

#include "pros/rtos.hpp"

#include <optional>

namespace autumn::motion_handler {

static std::optional<pros::Task> motionTask = std::nullopt;

void move(std::function<void()> motion) {
    while (isMoving()) {
        pros::delay(5);
    }
    setMotionCancelled(false);
    motionTask = pros::Task([motion] {
        if (pros::Task::notify_take(true, 0) == 0) {
            motion();
        }
    });
}

bool isMoving() {
    if (!motionTask.has_value()) {
        return false;
    }
    const std::uint32_t state = motionTask->get_state();
    return state != pros::E_TASK_STATE_DELETED && state != pros::E_TASK_STATE_INVALID;
}

void cancel() {
    setMotionCancelled(true);
    if (isMoving()) {
        motionTask->notify();
    }
}

void waitUntilPoint(Pose target, Length radius, std::function<Pose()> poseGetter) {
    while (isMoving()) {
        if (poseGetter().distance(target) <= radius) {
            return;
        }
        pros::delay(5);
    }
}

void waitUntilDistance(Length distance, std::function<Pose()> poseGetter) {
    const Pose start = poseGetter();
    while (isMoving()) {
        if (poseGetter().distance(start) >= distance) {
            return;
        }
        pros::delay(5);
    }
}

} // namespace autumn::motion_handler
