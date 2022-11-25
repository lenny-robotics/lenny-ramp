#include <lenny/samp/LinkAccelerationRegularizerObjective.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

LinkAccelerationRegularizerObjective::LinkAccelerationRegularizerObjective(const Plan& plan)
    : optimization::Objective("Link Acceleration Regularizer"), plan(plan) {}

double LinkAccelerationRegularizerObjective::computeValue(const Eigen::VectorXd& q) const {
    double value = 0.0;
    for (const LinkRegularizer& lr : plan.linkRegularizers) {
        auto computeGlobalPointForIndex = [&](const int& index) -> Eigen::Vector3d {
            const Eigen::VectorXd q_index = plan.getAgentStateForTrajectoryIndex(q, index);
            return plan.agent->computeGlobalPoint(q_index, lr.localCoordinates, lr.linkName);
        };

        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(lr.range.get().first), plan.getTrajectoryIndexForPercentage(lr.range.get().second)};
        Eigen::Vector3d p_im2 = computeGlobalPointForIndex(indexRange.first - 2);
        Eigen::Vector3d p_im1 = computeGlobalPointForIndex(indexRange.first - 1);
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            const Eigen::Vector3d p_i = computeGlobalPointForIndex(i);
            const Eigen::Vector3d acc = p_i - 2.0 * p_im1 + p_im2;
            value += 0.5 * acc.dot(acc.cwiseProduct(lr.weights));
            p_im2 = p_im1;
            p_im1 = p_i;
        }
    }
    return value;
}

void LinkAccelerationRegularizerObjective::computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const {
    pVpQ.resize(q.size());
    pVpQ.setZero();
    const uint stateSize = plan.agent->getStateSize();
    for (const LinkRegularizer& lr : plan.linkRegularizers) {
        auto computeGlobalPointForIndex = [&](const int& index) -> Eigen::Vector3d {
            const Eigen::VectorXd q_index = plan.getAgentStateForTrajectoryIndex(q, index);
            return plan.agent->computeGlobalPoint(q_index, lr.localCoordinates, lr.linkName);
        };

        auto computePointJacobianForIndex = [&](Eigen::MatrixXd& jacobian, const int& index) -> void {
            const Eigen::VectorXd q_index = plan.getAgentStateForTrajectoryIndex(q, index);
            plan.agent->computePointJacobian(jacobian, q_index, lr.localCoordinates, lr.linkName);
        };

        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(lr.range.get().first), plan.getTrajectoryIndexForPercentage(lr.range.get().second)};
        Eigen::Vector3d p_im2 = computeGlobalPointForIndex(indexRange.first - 2);
        Eigen::Vector3d p_im1 = computeGlobalPointForIndex(indexRange.first - 1);
        Eigen::MatrixXd J_im2, J_im1, J_i;
        if (indexRange.first - 2 >= 0) {
            computePointJacobianForIndex(J_im2, indexRange.first - 2);
        }
        if (indexRange.first - 1 >= 0) {
            computePointJacobianForIndex(J_im1, indexRange.first - 1);
        }
        for (int i = indexRange.first; i <= indexRange.second; i++) {
            const Eigen::Vector3d p_i = computeGlobalPointForIndex(i);
            const Eigen::Vector3d acc = p_i - 2.0 * p_im1 + p_im2;
            computePointJacobianForIndex(J_i, i);
            const Eigen::VectorXd vec = acc.cwiseProduct(lr.weights);

            pVpQ.segment(i * stateSize, stateSize) += J_i.transpose() * vec;
            if (i - 1 >= 0)
                pVpQ.segment((i - 1) * stateSize, stateSize) -= 2.0 * J_im1.transpose() * vec;
            if (i - 2 >= 0)
                pVpQ.segment((i - 2) * stateSize, stateSize) += J_im2.transpose() * vec;

            p_im2 = p_im1;
            p_im1 = p_i;
            J_im2 = J_im1;
            J_im1 = J_i;
        }
    }
}

void LinkAccelerationRegularizerObjective::computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const {
    p2VpQ2.resize(q.size(), q.size());
    Eigen::TripletDList tripletDList;
    const uint stateSize = plan.agent->getStateSize();
    for (const LinkRegularizer& lr : plan.linkRegularizers) {
        auto computeGlobalPointForIndex = [&](const int& index) -> Eigen::Vector3d {
            const Eigen::VectorXd q_index = plan.getAgentStateForTrajectoryIndex(q, index);
            return plan.agent->computeGlobalPoint(q_index, lr.localCoordinates, lr.linkName);
        };

        auto computePointJacobianForIndex = [&](Eigen::MatrixXd& jacobian, const int& index) -> void {
            const Eigen::VectorXd q_index = plan.getAgentStateForTrajectoryIndex(q, index);
            plan.agent->computePointJacobian(jacobian, q_index, lr.localCoordinates, lr.linkName);
        };

        auto computePointTensorForIndex = [&](Eigen::TensorD& tensor, const int& index) -> void {
            const Eigen::VectorXd q_index = plan.getAgentStateForTrajectoryIndex(q, index);
            plan.agent->computePointTensor(tensor, q_index, lr.localCoordinates, lr.linkName);
        };

        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(lr.range.get().first), plan.getTrajectoryIndexForPercentage(lr.range.get().second)};
        Eigen::Vector3d p_im2, p_im1, p_i;
        Eigen::MatrixXd J_im2, J_im1, J_i;
        Eigen::TensorD T_im2, T_im1, T_i;

        if (indexRange.first - 2 >= 0) {
            computePointJacobianForIndex(J_im2, indexRange.first - 2);
        }
        if (indexRange.first - 1 >= 0) {
            computePointJacobianForIndex(J_im1, indexRange.first - 1);
        }
        if (useRobotTensorForHessian || fdCheckIsBeingApplied) {
            p_im2 = computeGlobalPointForIndex(indexRange.first - 2);
            p_im1 = computeGlobalPointForIndex(indexRange.first - 1);
            if (indexRange.first - 2 >= 0) {
                computePointTensorForIndex(T_im2, indexRange.first - 2);
            }
            if (indexRange.first - 1 >= 0) {
                computePointTensorForIndex(T_im1, indexRange.first - 1);
            }
        }

        for (int i = indexRange.first; i <= indexRange.second; i++) {
            computePointJacobianForIndex(J_i, i);

            Eigen::MatrixXd H_i_i = (J_i.transpose() * lr.weights.asDiagonal() * J_i).triangularView<Eigen::Lower>();
            Eigen::MatrixXd H_i_im1, H_im1_im1, H_i_im2, H_im1_im2, H_im2_im2;
            if (i - 1 >= 0) {
                H_i_im1 = -2.0 * J_i.transpose() * lr.weights.asDiagonal() * J_im1;  //Full matrix needed!
                H_im1_im1 = (4.0 * J_im1.transpose() * lr.weights.asDiagonal() * J_im1).triangularView<Eigen::Lower>();
            }
            if (i - 2 >= 0) {
                H_i_im2 = J_i.transpose() * lr.weights.asDiagonal() * J_im2;             //Full matrix needed!
                H_im1_im2 = -2.0 * J_im1.transpose() * lr.weights.asDiagonal() * J_im2;  //Full matrix needed!
                H_im2_im2 = (J_im2.transpose() * lr.weights.asDiagonal() * J_im2).triangularView<Eigen::Lower>();
            }

            if (useRobotTensorForHessian || fdCheckIsBeingApplied) {
                p_i = computeGlobalPointForIndex(i);
                const Eigen::Vector3d acc = p_i - 2.0 * p_im1 + p_im2;
                const Eigen::VectorXd vec = acc.cwiseProduct(lr.weights);
                computePointTensorForIndex(T_i, i);

                Eigen::SparseMatrixD mat;
                T_i.multiply(mat, vec);
                H_i_i += mat.triangularView<Eigen::Lower>();

                if (i - 1 >= 0) {
                    T_im1.multiply(mat, vec);
                    H_im1_im1 -= 2.0 * mat.triangularView<Eigen::Lower>();
                }

                if (i - 2 >= 0) {
                    T_im2.multiply(mat, vec);
                    H_im2_im2 += mat.triangularView<Eigen::Lower>();
                }
            }

            auto addMatrixToTriplets = [&](const Eigen::MatrixXd& matrix, const int& rowIndex, const int& colIndex) -> void {
                for (int k = 0; k < matrix.outerSize(); ++k)
                    for (Eigen::MatrixXd::InnerIterator it(matrix, k); it; ++it)
                        tools::utils::addTripletDToList(tripletDList, rowIndex * stateSize + it.row(), colIndex * stateSize + it.col(), it.value());
            };

            addMatrixToTriplets(H_i_i, i, i);
            if (i - 1 >= 0) {
                addMatrixToTriplets(H_i_im1, i, i - 1);
                addMatrixToTriplets(H_im1_im1, i - 1, i - 1);
            }
            if (i - 2 >= 0) {
                addMatrixToTriplets(H_i_im2, i, i - 2);
                addMatrixToTriplets(H_im1_im2, i - 1, i - 2);
                addMatrixToTriplets(H_im2_im2, i - 2, i - 2);
            }

            if (useRobotTensorForHessian || fdCheckIsBeingApplied) {
                p_im2 = p_im1;
                p_im1 = p_i;
                T_im2 = T_im1;
                T_im1 = T_i;
            }
            J_im2 = J_im1;
            J_im1 = J_i;
        }
    }
    p2VpQ2.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void LinkAccelerationRegularizerObjective::drawGuiContent() {
    optimization::Objective::drawGuiContent();
    tools::Gui::I->Checkbox("Use Robot Tensor For Hessian", useRobotTensorForHessian);
}

}  // namespace lenny::samp