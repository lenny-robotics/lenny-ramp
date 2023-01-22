#pragma once

#include <lenny/optimization/InequalityConstraint.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class AccelerationLimitsConstraint : public optimization::InequalityConstraint {
public:
    AccelerationLimitsConstraint(const Plan& plan);
    ~AccelerationLimitsConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

public:
    double testFactor = 0.1; //ToDo: Does this make sense?

private:
    const Plan& plan;
};

}  // namespace lenny::samp