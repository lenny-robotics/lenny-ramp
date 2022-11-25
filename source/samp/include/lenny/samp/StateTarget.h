#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/Percentage.h>
#include <lenny/tools/Definitions.h>

namespace lenny::samp {

class StateTarget {
public:
    StateTarget(const rapt::Agent::CSPtr agent, const Eigen::VectorXd& state, const Eigen::VectorXd& weights, const double& step);
    ~StateTarget() = default;

    void setState(const Eigen::VectorXd& state);
    void setWeights(const Eigen::VectorXd& weights);

    const Eigen::VectorXd& getState() const;
    const Eigen::VectorXd& getWeights() const;

    void drawGui(const std::string& description);

private:
    const rapt::Agent::CSPtr agent;
    Eigen::VectorXd state;    //Target for full ROBOT state
    Eigen::VectorXd weights;  //Weights for individual ROBOT DOFs

public:
    Percentage step;  //Trajectory step when target is applied
};

}  // namespace lenny::samp