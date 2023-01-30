#include <lenny/samp/LinkLinearPositionLimitsConstraint.h>

namespace lenny::samp {

LinkLinearPositionLimitsConstraint::LinkLinearPositionLimitsConstraint(const Plan& plan)
    : LinkLimitsConstraint<Eigen::Vector3d>("Linear Position Limits", plan, 1.0, 0.1) {}

void LinkLinearPositionLimitsConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupLimitInfos(q, plan.linkPositionLimits, 1.0);
}

Eigen::Vector3d LinkLinearPositionLimitsConstraint::computeValue(const Eigen::VectorXd& q, const std::string& linkName, const Eigen::Vector3d& local,
                                                                 const uint& index) const {
    const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(index);
    return plan.agent->computeGlobalPoint(agentState, local, linkName);
}

void LinkLinearPositionLimitsConstraint::computeJacobian(Eigen::TripletDList& jacobian, const Eigen::VectorXd& q, const std::string& linkName,
                                                         const Eigen::Vector3d& local, const uint& index) const {
    const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(index);
    Eigen::MatrixXd dpdq;
    plan.agent->computePointJacobian(dpdq, agentState, local, linkName);
    jacobian.clear();
    for (int k = 0; k < dpdq.outerSize(); ++k)
        for (Eigen::MatrixXd::InnerIterator it(dpdq, k); it; ++it)
            tools::utils::addTripletDToList(jacobian, it.row(), it.col(), it.value());
}

void LinkLinearPositionLimitsConstraint::computeTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& q, const std::string& linkName,
                                                       const Eigen::Vector3d& local, const uint& index) const {
    const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(index);
    plan.agent->computePointTensor(tensor, agentState, local, linkName);
}

}  // namespace lenny::samp