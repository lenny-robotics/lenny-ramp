#include <lenny/samp/StateAccelerationRegularizerObjective.h>

namespace lenny::samp {

StateAccelerationRegularizerObjective::StateAccelerationRegularizerObjective(const Plan& plan)
    : optimization::Objective("State Acceleration Regularizer"), plan(plan) {}

double StateAccelerationRegularizerObjective::computeValue(const Eigen::VectorXd& q) const {
    double value = 0.0;
    for (const StateRegularizer& sr : plan.stateRegularizers) {
        const Eigen::VectorXd agentWeights = plan.agent->getAgentStateFromRobotState(sr.getWeights());
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(sr.range.get().first), plan.getTrajectoryIndexForPercentage(sr.range.get().second)};
        Eigen::VectorXd q_im2 = plan.getAgentStateForTrajectoryIndex(q, indexRange.first - 2);
        Eigen::VectorXd q_im1 = plan.getAgentStateForTrajectoryIndex(q, indexRange.first - 1);
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            const Eigen::VectorXd q_i = plan.getAgentStateForTrajectoryIndex(q, i);
            const Eigen::VectorXd acc = plan.agent->estimateAgentAcceleration(q_i, q_im1, q_im2, 1.0);
            value += 0.5 * acc.dot(acc.cwiseProduct(agentWeights));
            q_im2 = q_im1;
            q_im1 = q_i;
        }
    }
    return value;
}

void StateAccelerationRegularizerObjective::computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const {
    pVpQ.resize(q.size());
    pVpQ.setZero();
    const uint stateSize = plan.agent->getStateSize();
    for (const StateRegularizer& sr : plan.stateRegularizers) {
        const Eigen::VectorXd agentWeights = plan.agent->getAgentStateFromRobotState(sr.getWeights());
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(sr.range.get().first), plan.getTrajectoryIndexForPercentage(sr.range.get().second)};
        Eigen::VectorXd q_im2 = plan.getAgentStateForTrajectoryIndex(q, indexRange.first - 2);
        Eigen::VectorXd q_im1 = plan.getAgentStateForTrajectoryIndex(q, indexRange.first - 1);
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            const Eigen::VectorXd q_i = plan.getAgentStateForTrajectoryIndex(q, i);
            const Eigen::VectorXd acc = plan.agent->estimateAgentAcceleration(q_i, q_im1, q_im2, 1.0);
            const Eigen::VectorXd vec = acc.cwiseProduct(agentWeights);
            pVpQ.segment(i * stateSize, stateSize) += vec;
            if (i - 1 >= 0)
                pVpQ.segment((i - 1) * stateSize, stateSize) -= 2.0 * vec;
            if (i - 2 >= 0)
                pVpQ.segment((i - 2) * stateSize, stateSize) += vec;
            q_im2 = q_im1;
            q_im1 = q_i;
        }
    }
}

void StateAccelerationRegularizerObjective::computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const {
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
                    tools::utils::addTripletDToList(tripletDList, i * stateSize + j, (i - 1) * stateSize + j, -2.0 * agentWeights[j]);
                    tools::utils::addTripletDToList(tripletDList, (i - 1) * stateSize + j, (i - 1) * stateSize + j, 4.0 * agentWeights[j]);
                }
                if (i - 2 >= 0) {
                    tools::utils::addTripletDToList(tripletDList, i * stateSize + j, (i - 2) * stateSize + j, agentWeights[j]);
                    tools::utils::addTripletDToList(tripletDList, (i - 1) * stateSize + j, (i - 2) * stateSize + j, -2.0 * agentWeights[j]);
                    tools::utils::addTripletDToList(tripletDList, (i - 2) * stateSize + j, (i - 2) * stateSize + j, agentWeights[j]);
                }
            }
        }
    }
    p2VpQ2.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

}  // namespace lenny::samp