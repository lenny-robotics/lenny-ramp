#pragma once

#include <lenny/optimization/Objective.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class LinkAccelerationRegularizerObjective : public optimization::Objective {
public:
    LinkAccelerationRegularizerObjective(const Plan& plan);
    ~LinkAccelerationRegularizerObjective() = default;

    double computeValue(const Eigen::VectorXd& q) const override;
    void computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const override;
    void computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

public:
    bool useRobotTensorForHessian = false;

private:
    const Plan& plan;
};

}  // namespace lenny::samp