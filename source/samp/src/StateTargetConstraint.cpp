#include <lenny/samp/StateTargetConstraint.h>

namespace lenny::samp {

StateTargetConstraint::StateTargetConstraint(const Plan& plan) : optimization::EqualityConstraint("State Target"), plan(plan) {
    useTensorForHessian = false;
}

uint StateTargetConstraint::getConstraintNumber() const {
    return plan.agent->getStateSize() * plan.stateTargets.size();
}

void StateTargetConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    const uint stateSize = plan.agent->getStateSize();
    for (uint i = 0; i < plan.stateTargets.size(); i++) {
        const StateTarget& target = plan.stateTargets[i];
        const int index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        const Eigen::VectorXd targetState = plan.agent->getAgentStateFromRobotState(target.getState());
        const Eigen::VectorXd weights = plan.agent->getAgentStateFromRobotState(target.getWeights());
        C.segment(i * stateSize, stateSize) = weights.cwiseProduct(agentState - targetState);
    }
}

void StateTargetConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    Eigen::TripletDList tripletDList;
    const uint stateSize = plan.agent->getStateSize();
    for (uint i = 0; i < plan.stateTargets.size(); i++) {
        const StateTarget& target = plan.stateTargets[i];
        const int index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd weights = plan.agent->getAgentStateFromRobotState(target.getWeights());
        for (uint j = 0; j < stateSize; j++)
            tools::utils::addTripletDToList(tripletDList, i * stateSize + j, index * stateSize + j, weights[j]);
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void StateTargetConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

}  // namespace lenny::samp