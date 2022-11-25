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

void MotionTrajectoryHandler::getTrajectoryForAgent(Eigen::VectorXd& trajectory, uint& startIndex, const rapt::Agent::CSPtr agent,
                                                    const Eigen::VectorXd& stackedTrajectory) const {
    checkStackedTrajectory(stackedTrajectory);
    for (uint iter = 0; const samp::Plan& plan : plans) {
        const uint currentSize = plan.getNumSteps() * plan.agent->getStateSize();
        if (plan.agent == agent) {
            trajectory = stackedTrajectory.segment(iter, currentSize);
            startIndex = iter;
            return;
        }
        iter += currentSize;
    }
    LENNY_LOG_ERROR("There seems to be no motion plan for the agent with name `%s`", agent->name.c_str());
}

void MotionTrajectoryHandler::getAgentStatesForTrajectoryIndex(std::map<std::string, Eigen::VectorXd>& agentStates, const Eigen::VectorXd& stackedTrajectory,
                                                               const int& index) const {
    checkStackedTrajectory(stackedTrajectory);
    agentStates.clear();
    for (uint iter = 0; const samp::Plan& plan : plans) {
        const uint currentSize = plan.getNumSteps() * plan.agent->getStateSize();
        agentStates.insert({plan.agent->name, plan.getAgentStateForTrajectoryIndex(stackedTrajectory.segment(iter, currentSize), index)});
        iter += currentSize;
    }
}

void MotionTrajectoryHandler::getAgentStatesForTrajectoryPercentage(std::map<std::string, Eigen::VectorXd>& agentStates,
                                                                    const Eigen::VectorXd& stackedTrajectory, const double& percentage) const {
    checkStackedTrajectory(stackedTrajectory);
    agentStates.clear();
    for (uint iter = 0; const samp::Plan& plan : plans) {
        const uint currentSize = plan.getNumSteps() * plan.agent->getStateSize();
        agentStates.insert({plan.agent->name, plan.getAgentStateForTrajectoryPercentage(stackedTrajectory.segment(iter, currentSize), percentage)});
        iter += currentSize;
    }
}

void MotionTrajectoryHandler::getAgentStatesForTrajectoryTime(std::map<std::string, Eigen::VectorXd>& agentStates, const Eigen::VectorXd& stackedTrajectory,
                                                              const double& time) const {
    checkStackedTrajectory(stackedTrajectory);
    agentStates.clear();
    for (uint iter = 0; const samp::Plan& plan : plans) {
        const uint currentSize = plan.getNumSteps() * plan.agent->getStateSize();
        agentStates.insert({plan.agent->name, plan.getAgentStateForTrajectoryTime(stackedTrajectory.segment(iter, currentSize), time)});
        iter += currentSize;
    }
}

}  // namespace lenny::mamp