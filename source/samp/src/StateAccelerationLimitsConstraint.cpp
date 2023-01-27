#include <lenny/samp/StateAccelerationLimitsConstraint.h>

namespace lenny::samp {

StateAccelerationLimitsConstraint::StateAccelerationLimitsConstraint(const Plan& plan)
    : StateLimitsConstraint("State Acceleration Limits", plan, robot::Robot::ACCELERATION, 0.0001, 0.1) {}

double StateAccelerationLimitsConstraint::computeValue(const Eigen::VectorXd& q, const int& index) const {
    const int stateSize = plan.agent->getStateSize();
    const double q_im1 = (index - stateSize < 0) ? plan.agent->getInitialAgentState()[index] : q[index - stateSize];
    double q_im2;
    if (index - 2 * stateSize < 0 && index - stateSize < 0)
        q_im2 = plan.agent->getInitialAgentState()[index] - plan.getDeltaT() * plan.agent->getInitialAgentVelocity()[index];
    else if (index - 2 * stateSize < 0 && index - stateSize >= 0)
        q_im2 = plan.agent->getInitialAgentState()[index - stateSize];
    else
        q_im2 = q[index - 2 * stateSize];
    return robot::Robot::estimateAngularAcceleration(q[index], q_im1, q_im2, plan.getDeltaT());
}

std::vector<std::pair<double, int>> StateAccelerationLimitsConstraint::computeValueDerivative(const Eigen::VectorXd& q, const int& index) const {
    const int stateSize = plan.agent->getStateSize();
    std::vector<std::pair<double, int>> entries;
    if (index - 2 * stateSize >= 0)
        entries.push_back({1.0 / (plan.getDeltaT() * plan.getDeltaT()), index - 2 * stateSize});
    if (index - stateSize >= 0)
        entries.push_back({-2.0 / (plan.getDeltaT() * plan.getDeltaT()), index - stateSize});
    entries.push_back({1.0 / (plan.getDeltaT() * plan.getDeltaT()), index});
    return entries;
}

}  // namespace lenny::samp