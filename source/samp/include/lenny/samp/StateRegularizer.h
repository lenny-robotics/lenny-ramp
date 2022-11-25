#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/PercentageRange.h>
#include <lenny/tools/Definitions.h>

namespace lenny::samp {

class StateRegularizer {
public:
    StateRegularizer(const rapt::Agent::CSPtr agent, const Eigen::VectorXd& weights, const std::pair<double, double>& range);
    ~StateRegularizer() = default;

    void setWeights(const Eigen::VectorXd& weights);
    const Eigen::VectorXd& getWeights() const;

    void drawGui(const std::string& description);

private:
    const rapt::Agent::CSPtr agent;
    Eigen::VectorXd weights;  //Weights for individual ROBOT DOFs

public:
    PercentageRange range;  //Trajectory range when regularization is applied
};

}  // namespace lenny::samp