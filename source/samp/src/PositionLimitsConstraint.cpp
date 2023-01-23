#include <lenny/samp/PositionLimitsConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

PositionLimitsConstraint::PositionLimitsConstraint(const Plan& plan) : optimization::InequalityConstraint("Position Limits"), plan(plan) {
    useTensorForHessian = false;
    barrier.setStiffness(1.0);
    barrier.setEpsilon(0.0);
}

uint PositionLimitsConstraint::getConstraintNumber() const {
    return limitInfos.size();
}

void PositionLimitsConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();

    if (C.size() == 0)
        return;

    uint iter = 0;
    for (const auto& [index, limit, delta, type] : limitInfos) {
        const double q_index = q[index] - delta;
        if (type == LOWER)
            C[iter] = limit - q_index;
        else if (type == UPPER)
            C[iter] = q_index - limit;
        iter++;
    }
}

void PositionLimitsConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    Eigen::TripletDList tripletDList;
    uint iter = 0;
    for (const auto& [index, limit, delta, type] : limitInfos) {
        if (type == LOWER)
            tools::utils::addTripletDToList(tripletDList, iter, index, -1.0);
        else if (type == UPPER)
            tools::utils::addTripletDToList(tripletDList, iter, index, 1.0);
        iter++;
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void PositionLimitsConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

void PositionLimitsConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Test Factor", testFactor);
    tools::Gui::I->Checkbox("Print Limit Infos", printLimitInfos);
}

void PositionLimitsConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupLimitInfos(q);
}

void PositionLimitsConstraint::setupLimitInfos(const Eigen::VectorXd& q) const {
    //Clear list
    limitInfos.clear();

    //Compute delta state to account for local base transformation
    Eigen::VectorXd initialRobotState = plan.agent->getInitialRobotState();
    const tools::Transformation globalBasePose =
        plan.agent->robot.base->getTransformationFromState(initialRobotState.segment(0, 6)) * plan.agent->localBaseTrafo.inverse();
    initialRobotState.segment(0, 6) = plan.agent->robot.base->getStateFromTransformation(globalBasePose);
    const Eigen::VectorXd deltaState =
        plan.agent->getAgentStateFromRobotState(initialRobotState) - plan.agent->getInitialAgentState();  //ToDo: Test this carefully
    LENNY_LOG_DEBUG("%s", Eigen::to_string(deltaState).c_str())

    //Loop over state
    for (uint i = 0; i < deltaState.size(); i++) {
        //Get limit and check if it is active
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::POSITION);
        if (!limit.has_value())
            continue;

        //Loop over trajectory
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index = j * deltaState.size() + i;
            const double q_ij = q[index] - deltaState[i];

            //lower limit is active
            if (q_ij - limit->first < testFactor) {
                limitInfos.push_back({index, limit->first, deltaState[i], LOWER});
                if (printLimitInfos)
                    LENNY_LOG_DEBUG("Added LOWER position limit for joint '%s' at trajectory step '%d'", plan.agent->getDescriptionForDofIndex(i).c_str(), j)
            }

            //upper limit is active
            if (limit->second - q_ij < testFactor) {
                limitInfos.push_back({index, limit->second, deltaState[i], UPPER});
                if (printLimitInfos)
                    LENNY_LOG_DEBUG("Added UPPER position limit for joint '%s' at trajectory step '%d'", plan.agent->getDescriptionForDofIndex(i).c_str(), j)
            }
        }
    }
}

}  // namespace lenny::samp