#include <lenny/samp/StatePositionLimitsConstraint.h>

namespace lenny::samp {

StatePositionLimitsConstraint::StatePositionLimitsConstraint(const Plan& plan)
    : StateLimitsConstraint("State Position Limits", plan, robot::Robot::POSITION, 1.0, 0.1) {}

void StatePositionLimitsConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupLimitInfos(q, 1.0);
}

double StatePositionLimitsConstraint::computeValue(const Eigen::VectorXd& q, const int& index) const {
    return q[index];
}

std::vector<std::pair<double, int>> StatePositionLimitsConstraint::computeValueDerivative(const Eigen::VectorXd& q, const int& index) const {
    return {{1.0, index}};
}

}  // namespace lenny::samp