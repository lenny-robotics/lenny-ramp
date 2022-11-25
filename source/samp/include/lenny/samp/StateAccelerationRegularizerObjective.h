#pragma once

#include <lenny/optimization/Objective.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class StateAccelerationRegularizerObjective : public optimization::Objective {
public:
    StateAccelerationRegularizerObjective(const Plan& plan);
    ~StateAccelerationRegularizerObjective() = default;

    double computeValue(const Eigen::VectorXd& q) const override;
    void computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const override;
    void computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const override;

private:
    const Plan& plan;
};

}  // namespace lenny::samp