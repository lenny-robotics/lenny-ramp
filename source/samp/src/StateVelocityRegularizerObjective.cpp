#include <lenny/samp/StateVelocityRegularizerObjective.h>

namespace lenny::samp {

StateVelocityRegularizerObjective::StateVelocityRegularizerObjective(const Plan& plan)
    : optimization::Objective("State Velocity Regularizer"), plan(plan) {}

double StateVelocityRegularizerObjective::computeValue(const Eigen::VectorXd& q) const {
    double value = 0.0;
    for (const StateRegularizer& sr : plan.stateRegularizers) {
        const Eigen::VectorXd agentWeights = plan.agent->getAgentStateFromRobotState(sr.getWeights());
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(sr.range.get().first), plan.getTrajectoryIndexForPercentage(sr.range.get().second)};
        Eigen::VectorXd q_im1 = plan.getAgentStateForTrajectoryIndex(q, indexRange.first - 1);
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            const Eigen::VectorXd q_i = plan.getAgentStateForTrajectoryIndex(q, i);
            const Eigen::VectorXd vel = q_i - q_im1;
            value += 0.5 * vel.dot(vel.cwiseProduct(agentWeights));
            q_im1 = q_i;
        }
    }
    return value;
}

void StateVelocityRegularizerObjective::computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const {
    pVpQ.resize(q.size());
    pVpQ.setZero();
    const uint stateSize = plan.agent->getStateSize();
    for (const StateRegularizer& sr : plan.stateRegularizers) {
        const Eigen::VectorXd agentWeights = plan.agent->getAgentStateFromRobotState(sr.getWeights());
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(sr.range.get().first), plan.getTrajectoryIndexForPercentage(sr.range.get().second)};
        Eigen::VectorXd q_im1 = plan.getAgentStateForTrajectoryIndex(q, indexRange.first - 1);
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            const Eigen::VectorXd q_i = plan.getAgentStateForTrajectoryIndex(q, i);
            const Eigen::VectorXd vel = q_i - q_im1;
            const Eigen::VectorXd vec = vel.cwiseProduct(agentWeights);
            pVpQ.segment(i * stateSize, stateSize) += vec;
            if (i - 1 >= 0)
                pVpQ.segment((i - 1) * stateSize, stateSize) -= vec;
            q_im1 = q_i;
        }
    }
}

void StateVelocityRegularizerObjective::computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const {
    p2VpQ2.resize(q.size(), q.size());
    Eigen::TripletDList tripletDList;
    const uint stateSize = plan.agent->getStateSize();
    for (const StateRegularizer& sr : plan.stateRegularizers) {
        const Eigen::VectorXd agentWeights = plan.agent->getAgentStateFromRobotState(sr.getWeights());
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(sr.range.get().first), plan.getTrajectoryIndexForPercentage(sr.range.get().second)};
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            for (uint j = 0; j < stateSize; j++) {
                tools::utils::addTripletDToList(tripletDList, i * stateSize + j, i * stateSize + j, agentWeights[j]);
                if (i - 1 >= 0) {
                    tools::utils::addTripletDToList(tripletDList, i * stateSize + j, (i - 1) * stateSize + j, -agentWeights[j]);
                    tools::utils::addTripletDToList(tripletDList, (i - 1) * stateSize + j, (i - 1) * stateSize + j, agentWeights[j]);
                }
            }
        }
    }
    p2VpQ2.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

}  // namespace lenny::samp