#include <lenny/collision/Api.h>
#include <lenny/samp/SelfCollisionAvoidanceConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

SelfCollisionAvoidanceConstraint::SelfCollisionAvoidanceConstraint(const Plan& plan)
    : optimization::InequalityConstraint("Self Collision Avoidance"), plan(plan) {
    useTensorForHessian = false;
    barrier.setStiffness(1.0);
    barrier.setEpsilon(0.005);
}

uint SelfCollisionAvoidanceConstraint::getConstraintNumber() const {
    uint numC = 0;
    for (const auto& pair : pairList)
        numC += pair.second.size();
    return numC;
}

void SelfCollisionAvoidanceConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();

    if (C.size() == 0)
        return;

    uint iter = 0;
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        for (auto& [primitive_A, primitive_B, t] : pairs) {
            C[iter] = -1.0 * collision::Api::compute_D(t, {primitive_A, agentState}, {primitive_B, agentState});
            iter++;
        }
    }

    if (iter != C.size())
        LENNY_LOG_ERROR("Something is wrong with the constraint size")
}

void SelfCollisionAvoidanceConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    uint iter = 0;
    Eigen::TripletDList tripletDList;
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        const uint stateSize = agentState.size();
        const uint index = i * stateSize;
        for (auto& [primitive_A, primitive_B, t] : pairs) {
            Eigen::VectorXd dDdS;
            collision::Api::compute_dDdS(dDdS, t, {primitive_A, agentState}, {primitive_B, agentState});
            const Eigen::VectorXd dDdQ = dDdS.segment(0, stateSize) + dDdS.segment(stateSize, stateSize);
            for (uint j = 0; j < dDdQ.size(); j++)
                tools::utils::addTripletDToList(tripletDList, iter, index + j, -1.0 * dDdQ[j]);
            iter++;
        }
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void SelfCollisionAvoidanceConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();

    if (p2CpQ2.getDimensions()[0] == 0)
        return;

    uint iter = 0;
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        const uint stateSize = agentState.size();
        const uint index = i * stateSize;
        for (auto& [primitive_A, primitive_B, t] : pairs) {
            primitive_A->parent->useTensor = useParentTensor || fdCheckIsBeingApplied;
            primitive_B->parent->useTensor = useParentTensor || fdCheckIsBeingApplied;
            Eigen::MatrixXd d2DdS2;
            collision::Api::compute_d2DdS2(d2DdS2, t, {primitive_A, agentState}, {primitive_B, agentState});
            const Eigen::MatrixXd d2DdQ2 = d2DdS2.block(0, 0, stateSize, stateSize) + d2DdS2.block(0, stateSize, stateSize, stateSize) +
                                           d2DdS2.block(stateSize, 0, stateSize, stateSize) + d2DdS2.block(stateSize, stateSize, stateSize, stateSize);
            for (int k = 0; k < d2DdQ2.outerSize(); ++k)
                for (Eigen::MatrixXd::InnerIterator it(d2DdQ2, k); it; ++it)
                    p2CpQ2.addEntry(Eigen::Vector3i(iter, index + it.row(), index + it.col()), -1.0 * it.value());
            iter++;
        }
    }
}

void SelfCollisionAvoidanceConstraint::preFDEvaluation(const Eigen::VectorXd& q) const {
    updateTs(q, true);
}

bool SelfCollisionAvoidanceConstraint::preValueEvaluation(const Eigen::VectorXd& q) const {
    updateTs(q);
    return true;
}

void SelfCollisionAvoidanceConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupPairList(q);
}

void SelfCollisionAvoidanceConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Neighbor Radius", neighborRadius);
    tools::Gui::I->Checkbox("Use Parent Tensor", useParentTensor);
    tools::Gui::I->Checkbox("Print Primitive Pairs", printPrimitivePairs);
}

void SelfCollisionAvoidanceConstraint::setupPairList(const Eigen::VectorXd& q) const {
    if (printPrimitivePairs)
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "---------- Self Collision Avoidance Primitive Pairs ----------\n")

    pairList.clear();
    for (uint i = 0; i < plan.getNumSteps(); i++) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);

        for (const auto& [linkName_A, linkNameList] : plan.agent->selfCollisionLinkMap) {
            if (plan.agent->collisionPrimitives.find(linkName_A) == plan.agent->collisionPrimitives.end())
                continue;

            for (const auto primitive_A : plan.agent->collisionPrimitives.at(linkName_A)) {
                for (const auto& linkName_B : linkNameList) {
                    if (plan.agent->collisionPrimitives.find(linkName_B) == plan.agent->collisionPrimitives.end())
                        continue;

                    for (const auto primitive_B : plan.agent->collisionPrimitives.at(linkName_B)) {
                        Eigen::VectorXd t;
                        collision::Api::compute_T(t, {primitive_A, agentState}, {primitive_B, agentState});
                        const double D = collision::Api::compute_D(t, {primitive_A, agentState}, {primitive_B, agentState});

                        if (D < neighborRadius) {
                            if (pairList.find(i) == pairList.end())
                                pairList.insert({i, {}});
                            pairList.at(i).emplace_back(primitive_A, primitive_B, t);

                            if (printPrimitivePairs)
                                LENNY_LOG_PRINT(tools::Logger::DEFAULT,
                                                "- Trajectory step: %d / Agent parent A: %s / Agent primitive A: %s / Agent parent B: %s / Agent primitive B: "
                                                "%s / Distance: "
                                                "%lf VS %lf\n",
                                                i, primitive_A->parent->description.c_str(), primitive_A->description.c_str(),
                                                primitive_B->parent->description.c_str(), primitive_B->description.c_str(), D, neighborRadius)
                        }
                    }
                }
            }
        }
    }

    if (printPrimitivePairs)
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "-----------------------------------------------------------------\n")
}

void SelfCollisionAvoidanceConstraint::updateTs(const Eigen::VectorXd& q, const bool& forFD) const {
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        for (auto& [primitive_A, primitive_B, t] : pairs)
            collision::Api::compute_T(t, {primitive_A, agentState}, {primitive_B, agentState}, forFD);
    }
}

}  // namespace lenny::samp