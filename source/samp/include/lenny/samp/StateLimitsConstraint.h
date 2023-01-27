#pragma once

#include <lenny/optimization/InequalityConstraint.h>
#include <lenny/samp/Plan.h>

#include <functional>

namespace lenny::samp {

class StateLimitsConstraint : public optimization::InequalityConstraint {
public:
    StateLimitsConstraint(const std::string& description, const Plan& plan, const robot::Robot::LIMITS_TYPE& limitsType, const double& stiffness,
                          const double& testFactor);
    virtual ~StateLimitsConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

protected:
    void setupLimitInfos(const Eigen::VectorXd& q) const;

    virtual double computeValue(const Eigen::VectorXd& q, const int& index) const = 0;
    virtual std::vector<std::pair<double, int>> computeValueDerivative(const Eigen::VectorXd& q, const int& index) const = 0;

public:
    double testFactor;  //Set by constructor
    bool printLimitInfos = false;

protected:
    const Plan& plan;

    const robot::Robot::LIMITS_TYPE limitsType;  //Set by constructor
    enum BOUND_TYPE { LOWER, UPPER };
    typedef std::tuple<int, double, BOUND_TYPE> LimitInfo;  //[trajectoryIndex, limit, type]
    mutable std::vector<LimitInfo> limitInfos;
};

}  // namespace lenny::samp