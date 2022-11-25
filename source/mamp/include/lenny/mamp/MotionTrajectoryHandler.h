#pragma once

#include <lenny/samp/Plan.h>

namespace lenny::mamp {

class MotionTrajectoryHandler {
public:
    MotionTrajectoryHandler(std::vector<samp::Plan>& plans);
    ~MotionTrajectoryHandler() = default;

    uint getStackedTrajectorySize() const;
    void checkStackedTrajectory(const Eigen::VectorXd& stackedTrajectory) const;

    void assemble(Eigen::VectorXd& stackedTrajectory) const;
    void disassemble(const Eigen::VectorXd& stackedTrajectory);

    void getTrajectoryForAgent(Eigen::VectorXd& trajectory, uint& startIndex, const rapt::Agent::CSPtr agent, const Eigen::VectorXd& stackedTrajectory) const;

    //std::map<std::string, Eigen::VectorXd> -> [agent name, state]
    void getAgentStatesForTrajectoryIndex(std::map<std::string, Eigen::VectorXd>& agentStates, const Eigen::VectorXd& stackedTrajectory,
                                          const int& index) const;
    void getAgentStatesForTrajectoryPercentage(std::map<std::string, Eigen::VectorXd>& agentStates, const Eigen::VectorXd& stackedTrajectory,
                                               const double& percentage) const;
    void getAgentStatesForTrajectoryTime(std::map<std::string, Eigen::VectorXd>& agentStates, const Eigen::VectorXd& stackedTrajectory,
                                         const double& time) const;

public:
    std::vector<samp::Plan>& plans;
};

}  // namespace lenny::mamp