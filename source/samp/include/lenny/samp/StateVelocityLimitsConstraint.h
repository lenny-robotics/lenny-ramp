#pragma once

#include <lenny/samp/StateLimitsConstraint.h>

namespace lenny::samp {

class StateVelocityLimitsConstraint : public StateLimitsConstraint {
public:
    StateVelocityLimitsConstraint(const Plan& plan);
    ~StateVelocityLimitsConstraint() = default;

private:
    double computeValue(const Eigen::VectorXd& q, const int& index) const override;
    std::vector<std::pair<double, int>> computeValueDerivative(const Eigen::VectorXd& q, const int& index) const override;
};

}  // namespace lenny::samp