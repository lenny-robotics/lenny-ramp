#include <lenny/collision/Api.h>
#include <lenny/samp/WorldCollisionAvoidanceConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

WorldCollisionAvoidanceConstraint::WorldCollisionAvoidanceConstraint(const Plan& plan, const rapt::WorldCollisionHandler::PrimitiveList& worldPrimitives)
    : optimization::InequalityConstraint("World Collision Avoidance"), plan(plan), worldPrimitives(worldPrimitives) {
    useTensorForHessian = false;
    barrier.setStiffness(1.0);
    barrier.setEpsilon(0.005);
}

uint WorldCollisionAvoidanceConstraint::getConstraintNumber() const {
    uint numC = 0;
    for (const auto& pair : pairList)
        numC += pair.second.size();
    return numC;
}

void WorldCollisionAvoidanceConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();

    if (C.size() == 0)
        return;

    uint iter = 0;
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        for (auto& [agentPrimitive, worldPrimitivePair, t] : pairs) {
            C[iter] = -1.0 * collision::Api::compute_D(t, {agentPrimitive, agentState}, worldPrimitivePair);
            iter++;
        }
    }

    if (iter != C.size())
        LENNY_LOG_ERROR("Something is wrong with the constraint size")
}

void WorldCollisionAvoidanceConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    uint iter = 0;
    Eigen::TripletDList tripletDList;
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        const uint index = i * agentState.size();
        for (auto& [agentPrimitive, worldPrimitivePair, t] : pairs) {
            Eigen::VectorXd dDdQ;
            collision::Api::compute_dDdS(dDdQ, t, {agentPrimitive, agentState}, worldPrimitivePair);
            for (uint j = 0; j < dDdQ.size(); j++)
                tools::utils::addTripletDToList(tripletDList, iter, index + j, -1.0 * dDdQ[j]);
            iter++;
        }
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void WorldCollisionAvoidanceConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();

    if (p2CpQ2.getDimensions()[0] == 0)
        return;

    uint iter = 0;
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        const uint index = i * agentState.size();
        for (auto& [agentPrimitive, worldPrimitivePair, t] : pairs) {
            agentPrimitive->parent->useTensor = useParentTensor || fdCheckIsBeingApplied;
            Eigen::MatrixXd d2DdQ2;
            collision::Api::compute_d2DdS2(d2DdQ2, t, {agentPrimitive, agentState}, worldPrimitivePair);
            for (int k = 0; k < d2DdQ2.outerSize(); ++k)
                for (Eigen::MatrixXd::InnerIterator it(d2DdQ2, k); it; ++it)
                    p2CpQ2.addEntry(Eigen::Vector3i(iter, index + it.row(), index + it.col()), -1.0 * it.value());
            iter++;
        }
    }
}

void WorldCollisionAvoidanceConstraint::preValueEvaluation(const Eigen::VectorXd& q) const {
    updateTs(q);
}

void WorldCollisionAvoidanceConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupPairList(q);
}

void WorldCollisionAvoidanceConstraint::preFDEvaluation(const Eigen::VectorXd& q) const {
    updateTs(q, true);
}

void WorldCollisionAvoidanceConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Neighbor Radius", neighborRadius);
    tools::Gui::I->Checkbox("Use Parent Tensor", useParentTensor);
    tools::Gui::I->Checkbox("Print Primitive Pairs", printPrimitivePairs);
}

void WorldCollisionAvoidanceConstraint::setupPairList(const Eigen::VectorXd& q) const {
    if (printPrimitivePairs)
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "---------- World Collision Avoidance Primitive Pairs ----------\n")

    pairList.clear();
    for (uint i = 0; i < plan.getNumSteps(); i++) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        for (const auto& [linkName, agentPrimitives] : plan.agent->collisionPrimitives) {
            for (const auto agentPrimitive : agentPrimitives) {
                for (const auto& worldPrimitivePair : worldPrimitives) {
                    Eigen::VectorXd t;
                    collision::Api::compute_T(t, {agentPrimitive, agentState}, worldPrimitivePair);
                    const double D = collision::Api::compute_D(t, {agentPrimitive, agentState}, worldPrimitivePair);
                    if (D < neighborRadius) {
                        if (pairList.find(i) == pairList.end())
                            pairList.insert({i, {}});
                        pairList.at(i).emplace_back(agentPrimitive, worldPrimitivePair, t);
                        if (printPrimitivePairs)
                            LENNY_LOG_PRINT(tools::Logger::DEFAULT,
                                            "- Trajectory step: %d / Agent parent: %s / Agent primitive: %s / World Primitive: %s / Distance: %lf VS %lf\n", i,
                                            agentPrimitive->parent->description.c_str(), agentPrimitive->description.c_str(),
                                            worldPrimitivePair.first->description.c_str(), D, neighborRadius)
                    }
                }
            }
        }
    }

    if (printPrimitivePairs)
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "-----------------------------------------------------------------\n")
}

void WorldCollisionAvoidanceConstraint::updateTs(const Eigen::VectorXd& q, const bool& forFD) const {
    for (auto& [i, pairs] : pairList) {
        const Eigen::VectorXd agentState = plan.getAgentStateForTrajectoryIndex(q, i);
        for (auto& [agentPrimitive, worldPrimitivePair, t] : pairs)
            collision::Api::compute_T(t, {agentPrimitive, agentState}, worldPrimitivePair, forFD);
    }
}

}  // namespace lenny::samp