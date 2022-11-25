#include <lenny/mamp/MotionTrajectoryHandler.h>

namespace lenny::mamp {

MotionTrajectoryHandler::MotionTrajectoryHandler(std::vector<samp::Plan>& plans) : plans(plans) {}

uint MotionTrajectoryHandler::getStackedTrajectorySize() const {
    uint size = 0;
    for (const samp::Plan& plan : plans)
        size += plan.getNumSteps() * plan.agent->getStateSize();
    return size;
}

void MotionTrajectoryHandler::checkStackedTrajectory(const Eigen::VectorXd& stackedTrajectory) const {
    const uint size = getStackedTrajectorySize();
    if (size != stackedTrajectory.size())
        LENNY_LOG_ERROR("Invalid input (Size: %d VS %d)", size, stackedTrajectory.size())
}

void MotionTrajectoryHandler::assemble(Eigen::VectorXd& stackedTrajectory) const {
    const uint size = getStackedTrajectorySize();
    stackedTrajectory.resize(size);
    for (uint iter = 0; const samp::Plan& plan : plans) {
        const uint currentSize = plan.getNumSteps() * plan.agent->getStateSize();
        stackedTrajectory.segment(iter, currentSize) = plan.motionTrajectory;
        iter += currentSize;
    }
}

void MotionTrajectoryHandler::disassemble(const Eigen::VectorXd& stackedTrajectory) {
    checkStackedTrajectory(stackedTrajectory);
    for (uint iter = 0; samp::Plan & plan : plans) {
        const uint currentSize = plan.getNumSteps() * plan.agent->getStateSize();
        plan.motionTrajectory = stackedTrajectory.segment(iter, currentSize);
        iter += currentSize;
    }
}

void MotionTrajectoryHandler::getAgentTrajectoryIndices(std::unordered_map<std::string, std::pair<uint, uint>>& trajectoryIndices) const {
    trajectoryIndices.clear();
    for (uint iter = 0; const samp::Plan& plan : plans) {
        const uint size = plan.getNumSteps() * plan.agent->getStateSize();
        trajectoryIndices.insert({plan.agent->name, {iter, size}});
        iter += size;
    }
}

}  // namespace lenny::mamp