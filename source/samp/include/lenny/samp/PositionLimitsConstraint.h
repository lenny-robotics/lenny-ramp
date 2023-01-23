#pragma once

#include <lenny/optimization/InequalityConstraint.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class PositionLimitsConstraint : public optimization::InequalityConstraint {
public:
    PositionLimitsConstraint(const Plan& plan);
    ~PositionLimitsConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

private:
    void setupLimitInfos(const Eigen::VectorXd& q) const;

public:
    double testFactor = 0.1;
    bool printLimitInfos = false;

private:
    const Plan& plan;

    enum LIMIT_TYPE { LOWER, UPPER };
    typedef std::tuple<uint, double, double, LIMIT_TYPE> LimitInfo;  //[trajectoryIndex, limit, delta, type]
    mutable std::vector<LimitInfo> limitInfos;
};

}  // namespace lenny::samp