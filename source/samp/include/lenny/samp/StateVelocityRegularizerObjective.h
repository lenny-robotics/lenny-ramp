#pragma once

#include <lenny/optimization/Objective.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class StateVelocityRegularizerObjective : public optimization::Objective {
public:
    StateVelocityRegularizerObjective(const Plan& plan);
    ~StateVelocityRegularizerObjective() = default;

    double computeValue(const Eigen::VectorXd& q) const override;
    void computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const override;
    void computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const override;

private:
    const Plan& plan;
};

}  // namespace lenny::samp