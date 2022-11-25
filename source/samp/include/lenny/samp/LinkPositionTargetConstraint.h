#pragma once

#include <lenny/optimization/EqualityConstraint.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class LinkPositionTargetConstraint : public optimization::EqualityConstraint {
public:
    LinkPositionTargetConstraint(const Plan& plan);
    ~LinkPositionTargetConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

private:
    const Plan& plan;
};

}  // namespace lenny::samp