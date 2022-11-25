#include <lenny/samp/LinkOrientationTargetConstraint.h>

namespace lenny::samp {

LinkOrientationTargetConstraint::LinkOrientationTargetConstraint(const Plan& plan)
    : optimization::EqualityConstraint("Link Orientation Target"), plan(plan) {
    useTensorForHessian = false;
}

uint LinkOrientationTargetConstraint::getConstraintNumber() const {
    uint numC = 0;
    for (const LinkTarget& target : plan.linkTargets)
        if (target.orientation.has_value())
            numC += 3;
    return numC;
}

void LinkOrientationTargetConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    int iter = 0;
    for (const LinkTarget& target : plan.linkTargets) {
        if (!target.orientation.has_value())
            continue;

        const uint index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        for (int i = 0; i < 3; i++) {
            const Eigen::Vector3d globalVector = plan.agent->computeGlobalVector(agentState, target.orientation->local.matrix().col(i), target.linkName);
            C[iter++] = target.orientation->weights[i] * (globalVector.dot(target.orientation->global.matrix().col(i)) - 1.0);
        }
    }
}

void LinkOrientationTargetConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    Eigen::TripletDList tripletDList;
    int iter = 0;
    for (const LinkTarget& target : plan.linkTargets) {
        if (!target.orientation.has_value())
            continue;

        const uint index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        const uint tIndex = index * agentState.size();
        for (int i = 0; i < 3; i++) {
            Eigen::MatrixXd jacobian;
            plan.agent->computeVectorJacobian(jacobian, agentState, target.orientation->local.matrix().col(i), target.linkName);
            const Eigen::VectorXd vec = target.orientation->weights[i] * jacobian.transpose() * target.orientation->global.matrix().col(i);
            for (int j = 0; j < vec.size(); j++)
                tools::utils::addTripletDToList(tripletDList, iter, tIndex + j, vec[j]);
            iter++;
        }
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void LinkOrientationTargetConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    int iter = 0;
    for (const LinkTarget& target : plan.linkTargets) {
        if (!target.orientation.has_value())
            continue;

        const uint index = plan.getTrajectoryIndexForPercentage(target.step.get());
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, index);
        const uint tIndex = index * agentState.size();
        for (int i = 0; i < 3; i++) {
            Eigen::TensorD tensor;
            plan.agent->computeVectorTensor(tensor, agentState, target.orientation->local.matrix().col(i), target.linkName);
            Eigen::SparseMatrixD mat;
            tensor.multiply(mat, target.orientation->global.matrix().col(i));
            for (int k = 0; k < mat.outerSize(); ++k)
                for (Eigen::SparseMatrixD::InnerIterator it(mat, k); it; ++it)
                    p2CpQ2.addEntry(Eigen::Vector3i(iter, tIndex + it.row(), tIndex + it.col()), target.orientation->weights[i] * it.value());
            iter++;
        }
    }
}
}  // namespace lenny::samp