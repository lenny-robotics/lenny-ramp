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

    //[agentName, [startIndex, size]]
    void getAgentTrajectoryIndices(std::unordered_map<std::string, std::pair<uint, uint>>& trajectoryIndices) const;

public:
    std::vector<samp::Plan>& plans;
};

}  // namespace lenny::mamp