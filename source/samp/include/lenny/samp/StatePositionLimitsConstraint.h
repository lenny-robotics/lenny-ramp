#pragma once

#include <lenny/samp/StateLimitsConstraint.h>

namespace lenny::samp {

class StatePositionLimitsConstraint : public StateLimitsConstraint {
public:
    StatePositionLimitsConstraint(const Plan& plan);
    ~StatePositionLimitsConstraint() = default;

private:
    double computeValue(const Eigen::VectorXd& q, const int& index) const override;
    std::vector<std::pair<double, int>> computeValueDerivative(const Eigen::VectorXd& q, const int& index) const override;
};

}  // namespace lenny::samp