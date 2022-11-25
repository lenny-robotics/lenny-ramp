#include <lenny/samp/LinkPositionTargetConstraint.h>

namespace lenny::samp {

LinkPositionTargetConstraint::LinkPositionTargetConstraint(const Plan& plan) : optimization::EqualityConstraint("Link Position Target"), plan(plan) {
    useTensorForHessian = false;
    constraintCheckTolerance = 1e-3;
}

uint LinkPositionTargetConstraint::getConstraintNumber() const {
    uint numC = 0;
    for (const LinkTarget& target : plan.linkTargets)
        if (target.position.has_value())
            numC += 3;
    return numC;
}

void LinkPositionTargetConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    int iter = 0;
    for (const LinkTarget& target : plan.linkTargets) {
        if (!target.position.has_value())
            continue;

        const uint index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        const Eigen::Vector3d globalPoint = plan.agent->computeGlobalPoint(agentState, target.position->local, target.linkName);
        C.segment(iter, 3) = target.position->weights.cwiseProduct(globalPoint - target.position->global);
        iter += 3;
    }
}

void LinkPositionTargetConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    Eigen::TripletDList tripletDList;
    int iter = 0;
    for (const LinkTarget& target : plan.linkTargets) {
        if (!target.position.has_value())
            continue;

        const uint index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        Eigen::MatrixXd jacobian;
        plan.agent->computePointJacobian(jacobian, agentState, target.position->local, target.linkName);
        const uint tIndex = index * plan.agent->getStateSize();
        for (int k = 0; k < jacobian.outerSize(); ++k)
            for (Eigen::MatrixXd::InnerIterator it(jacobian, k); it; ++it)
                tools::utils::addTripletDToList(tripletDList, iter + it.row(), tIndex + it.col(), target.position->weights[it.row()] * it.value());
        iter += 3;
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void LinkPositionTargetConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    int iter = 0;
    for (const LinkTarget& target : plan.linkTargets) {
        if (!target.position.has_value())
            continue;

        const uint index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        Eigen::TensorD tensor;
        plan.agent->computePointTensor(tensor, agentState, target.position->local, target.linkName);
        std::vector<std::pair<Eigen::Vector3i, double>> entryList;
        tensor.getEntryList(entryList);
        const uint tIndex = index * plan.agent->getStateSize();
        for (const auto& [ind, value] : entryList)
            p2CpQ2.addEntry(ind + Eigen::Vector3i(iter, tIndex, tIndex), target.position->weights[ind[0]] * value);
        iter += 3;
    }
}

}  // namespace lenny::samp