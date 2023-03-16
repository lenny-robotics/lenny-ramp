#include <lenny/samp/StateVelocityLimitsConstraint.h>

namespace lenny::samp {

StateVelocityLimitsConstraint::StateVelocityLimitsConstraint(const Plan& plan)
    : StateLimitsConstraint("State Velocity Limits", plan, robot::Robot::VELOCITY, 0.1, 1.0) {}

void StateVelocityLimitsConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupLimitInfos(q, plan.getDeltaT());
}

double StateVelocityLimitsConstraint::computeValue(const Eigen::VectorXd& q, const int& index) const {
    const int stateSize = plan.agent->getStateSize();
    if (index - stateSize < 0) {
        const Eigen::VectorXd q0 = plan.agent->getInitialAgentState();
        return robot::Robot::estimateAngularVelocity(q[index], q0[index], plan.getDeltaT());
    }
    return robot::Robot::estimateAngularVelocity(q[index], q[index - stateSize], plan.getDeltaT());
}

std::vector<std::pair<double, int>> StateVelocityLimitsConstraint::computeValueDerivative(const Eigen::VectorXd& q, const int& index) const {
    const int stateSize = plan.agent->getStateSize();
    std::vector<std::pair<double, int>> entries;
    if (index - stateSize >= 0)
        entries.push_back({-1.0 / plan.getDeltaT(), index - stateSize});
    entries.push_back({1.0 / plan.getDeltaT(), index});
    return entries;
}

}  // namespace lenny::samp