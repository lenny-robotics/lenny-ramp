#include <lenny/collision/Api.h>
#include <lenny/mamp/AgentCollisionAvoidanceConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::mamp {

AgentCollisionAvoidanceConstraint::AgentCollisionAvoidanceConstraint(const std::vector<samp::Plan>& plans)
    : optimization::InequalityConstraint("Agent Collision Avoidance"), plans(plans) {
    useTensorForHessian = false;
    barrier.setStiffness(1.0);
    barrier.setEpsilon(0.005);
}

uint AgentCollisionAvoidanceConstraint::getConstraintNumber() const {
    return pairList.size();
}

void AgentCollisionAvoidanceConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();

    if (C.size() == 0)
        return;

    uint iter = 0;
    for (auto& [primitive_A, primitive_B, t] : pairList) {
        const auto& [planIndex_A, trajStep_A, colPrim_A] = primitive_A;
        const auto [startIndex_A, size_A] = getInfosForPlanIndex(planIndex_A);
        const Eigen::VectorXd q_A = q.segment(startIndex_A, size_A);
        const Eigen::VectorXd agentState_A = plans.at(planIndex_A).getAgentStateForTrajectoryIndex(q_A, trajStep_A);

        const auto& [planIndex_B, trajStep_B, colPrim_B] = primitive_B;
        const auto [startIndex_B, size_B] = getInfosForPlanIndex(planIndex_B);
        const Eigen::VectorXd q_B = q.segment(startIndex_B, size_B);
        const Eigen::VectorXd agentState_B = plans.at(planIndex_B).getAgentStateForTrajectoryIndex(q_B, trajStep_B);

        C[iter] = -1.0 * collision::Api::compute_D(t, {colPrim_A, agentState_A}, {colPrim_B, agentState_B});
        iter++;
    }

    if (iter != C.size())
        LENNY_LOG_ERROR("Something is wrong with the constraint size")
}

void AgentCollisionAvoidanceConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    uint iter = 0;
    Eigen::TripletDList tripletDList;
    for (auto& [primitive_A, primitive_B, t] : pairList) {
        const auto& [planIndex_A, trajStep_A, colPrim_A] = primitive_A;
        const auto [startIndex_A, size_A] = getInfosForPlanIndex(planIndex_A);
        const Eigen::VectorXd q_A = q.segment(startIndex_A, size_A);
        const Eigen::VectorXd agentState_A = plans.at(planIndex_A).getAgentStateForTrajectoryIndex(q_A, trajStep_A);
        const uint stateSize_A = agentState_A.size();
        const uint trajIndex_A = startIndex_A + trajStep_A * stateSize_A;

        const auto& [planIndex_B, trajStep_B, colPrim_B] = primitive_B;
        const auto [startIndex_B, size_B] = getInfosForPlanIndex(planIndex_B);
        const Eigen::VectorXd q_B = q.segment(startIndex_B, size_B);
        const Eigen::VectorXd agentState_B = plans.at(planIndex_B).getAgentStateForTrajectoryIndex(q_B, trajStep_B);
        const uint stateSize_B = agentState_B.size();
        const uint trajIndex_B = startIndex_B + trajStep_B * stateSize_B;

        Eigen::VectorXd dDdS;
        collision::Api::compute_dDdS(dDdS, t, {colPrim_A, agentState_A}, {colPrim_B, agentState_B});
        for (uint j = 0; j < stateSize_A; j++)
            tools::utils::addTripletDToList(tripletDList, iter, trajIndex_A + j, -1.0 * dDdS[j]);
        for (uint j = 0; j < stateSize_B; j++)
            tools::utils::addTripletDToList(tripletDList, iter, trajIndex_B + j, -1.0 * dDdS[stateSize_A + j]);
        iter++;
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void AgentCollisionAvoidanceConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();

    if (p2CpQ2.getDimensions()[0] == 0)
        return;

    uint iter = 0;
    for (auto& [primitive_A, primitive_B, t] : pairList) {
        const auto& [planIndex_A, trajStep_A, colPrim_A] = primitive_A;
        const auto [startIndex_A, size_A] = getInfosForPlanIndex(planIndex_A);
        const Eigen::VectorXd q_A = q.segment(startIndex_A, size_A);
        const Eigen::VectorXd agentState_A = plans.at(planIndex_A).getAgentStateForTrajectoryIndex(q_A, trajStep_A);
        const uint stateSize_A = agentState_A.size();
        const uint trajIndex_A = startIndex_A + trajStep_A * stateSize_A;

        const auto& [planIndex_B, trajStep_B, colPrim_B] = primitive_B;
        const auto [startIndex_B, size_B] = getInfosForPlanIndex(planIndex_B);
        const Eigen::VectorXd q_B = q.segment(startIndex_B, size_B);
        const Eigen::VectorXd agentState_B = plans.at(planIndex_B).getAgentStateForTrajectoryIndex(q_B, trajStep_B);
        const uint stateSize_B = agentState_B.size();
        const uint trajIndex_B = startIndex_B + trajStep_B * stateSize_B;

        colPrim_A->parent->useTensor = useParentTensor || fdCheckIsBeingApplied;
        colPrim_B->parent->useTensor = useParentTensor || fdCheckIsBeingApplied;

        Eigen::MatrixXd d2DdS2;
        collision::Api::compute_d2DdS2(d2DdS2, t, {colPrim_A, agentState_A}, {colPrim_B, agentState_B});
        for (int k = 0; k < d2DdS2.outerSize(); ++k) {
            for (Eigen::MatrixXd::InnerIterator it(d2DdS2, k); it; ++it) {
                const uint row = (it.row() < stateSize_A) ? (trajIndex_A + it.row()) : (trajIndex_B + it.row() - stateSize_A);
                const uint col = (it.col() < stateSize_A) ? (trajIndex_A + it.col()) : (trajIndex_B + it.col() - stateSize_A);
                p2CpQ2.addEntry(Eigen::Vector3i(iter, row, col), -1.0 * it.value());
            }
        }
        iter++;
    }
}

void AgentCollisionAvoidanceConstraint::preFDEvaluation(const Eigen::VectorXd& q) const {
    updateTs(q, true);
}

bool AgentCollisionAvoidanceConstraint::preValueEvaluation(const Eigen::VectorXd& q) const {
    updateTs(q);
    return true;
}

void AgentCollisionAvoidanceConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupPairList(q);
}

void AgentCollisionAvoidanceConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Neighbor Radius", neighborRadius);
    tools::Gui::I->Checkbox("Use Parent Tensor", useParentTensor);
    tools::Gui::I->Checkbox("Print Primitive Pairs", printPrimitivePairs);
}

std::pair<uint, uint> AgentCollisionAvoidanceConstraint::getInfosForPlanIndex(const uint& planIndex) const {
    if (planIndex >= plans.size())
        LENNY_LOG_ERROR("Invalid plan index: %d VS %d", planIndex, plans.size())

    uint index = 0;
    uint size = 0;
    for (uint i = 0; i <= planIndex; i++) {
        const samp::Plan& plan = plans.at(i);
        size = plan.agent->getStateSize() * plan.getNumSteps();
        index += size;
    }
    return {index - size, size};
}

uint AgentCollisionAvoidanceConstraint::getTrajectoryStepForPlanIndex(const uint& planIndex, const std::pair<uint, uint>& otherPlanInfo) const {
    const auto& [otherPlanIndex, otherTrajectoryStep] = otherPlanInfo;
    const samp::Plan& plan = plans.at(planIndex);
    const samp::Plan& otherPlan = plans.at(otherPlanIndex);
    const double otherTime = (double)(otherTrajectoryStep + 1) * otherPlan.getDeltaT();
    const double totalTime = (double)(plan.getNumSteps() + 1) * plan.getDeltaT();
    return plan.getTrajectoryIndexForPercentage(otherTime / totalTime);
}

void AgentCollisionAvoidanceConstraint::setupPairList(const Eigen::VectorXd& q) const {
    if (printPrimitivePairs)
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "---------- Agent Collision Avoidance Primitive Pairs ----------\n")

    auto addPrimitivesToList = [&](const uint& planIndex_A, const uint& planIndex_B) -> void {
        const samp::Plan& plan_A = plans.at(planIndex_A);
        const samp::Plan& plan_B = plans.at(planIndex_B);

        for (uint trajStep_A = 0; trajStep_A < plan_A.getNumSteps(); trajStep_A++) {
            const auto [startIndex_A, size_A] = getInfosForPlanIndex(planIndex_A);
            const Eigen::VectorXd q_A = q.segment(startIndex_A, size_A);
            const Eigen::VectorXd agentState_A = plans.at(planIndex_A).getAgentStateForTrajectoryIndex(q_A, trajStep_A);

            const uint trajStep_B = getTrajectoryStepForPlanIndex(planIndex_B, {planIndex_A, trajStep_A});
            const auto [startIndex_B, size_B] = getInfosForPlanIndex(planIndex_B);
            const Eigen::VectorXd q_B = q.segment(startIndex_B, size_B);
            const Eigen::VectorXd agentState_B = plans.at(planIndex_B).getAgentStateForTrajectoryIndex(q_B, trajStep_B);

            for (const auto& [linkName_A, colPrims_A] : plan_A.agent->collisionPrimitives) {
                for (const auto colPrim_A : colPrims_A) {
                    for (const auto& [linkName_B, colPrims_B] : plan_B.agent->collisionPrimitives) {
                        for (const auto colPrim_B : colPrims_B) {
                            Eigen::VectorXd t;
                            collision::Api::compute_T(t, {colPrim_A, agentState_A}, {colPrim_B, agentState_B});
                            const double D = collision::Api::compute_D(t, {colPrim_A, agentState_A}, {colPrim_B, agentState_B});

                            if (D < neighborRadius) {
                                pairList.push_back({{planIndex_A, trajStep_A, colPrim_A}, {planIndex_B, trajStep_B, colPrim_B}, t});

                                if (printPrimitivePairs)
                                    LENNY_LOG_PRINT(
                                        tools::Logger::DEFAULT,
                                        "- Trajectory step A: %d / Agent parent A: %s / Agent primitive A: %s --- Trajectory step B: %d / Agent parent "
                                        "B: %s / Agent primitive B: "
                                        "%s / Distance: "
                                        "%lf VS %lf\n",
                                        trajStep_A, colPrim_A->parent->description.c_str(), colPrim_A->description.c_str(), trajStep_B,
                                        colPrim_B->parent->description.c_str(), colPrim_B->description.c_str(), D, neighborRadius)
                            }
                        }
                    }
                }
            }
        }
    };

    pairList.clear();
    for (uint i = 0; i < plans.size(); i++) {
        for (uint j = i + 1; j < plans.size(); j++) {
            addPrimitivesToList(i, j);
            addPrimitivesToList(j, i);
        }
    }

    if (printPrimitivePairs)
        LENNY_LOG_PRINT(tools::Logger::DEFAULT, "-----------------------------------------------------------------\n")
}

void AgentCollisionAvoidanceConstraint::updateTs(const Eigen::VectorXd& q, const bool& forFD) const {
    for (auto& [primitive_A, primitive_B, t] : pairList) {
        const auto& [planIndex_A, trajStep_A, colPrim_A] = primitive_A;
        const auto [startIndex_A, size_A] = getInfosForPlanIndex(planIndex_A);
        const Eigen::VectorXd q_A = q.segment(startIndex_A, size_A);
        const Eigen::VectorXd agentState_A = plans.at(planIndex_A).getAgentStateForTrajectoryIndex(q_A, trajStep_A);

        const auto& [planIndex_B, trajStep_B, colPrim_B] = primitive_B;
        const auto [startIndex_B, size_B] = getInfosForPlanIndex(planIndex_B);
        const Eigen::VectorXd q_B = q.segment(startIndex_B, size_B);
        const Eigen::VectorXd agentState_B = plans.at(planIndex_B).getAgentStateForTrajectoryIndex(q_B, trajStep_B);

        collision::Api::compute_T(t, {colPrim_A, agentState_A}, {colPrim_B, agentState_B}, forFD);
    }
}

}  // namespace lenny::mamp