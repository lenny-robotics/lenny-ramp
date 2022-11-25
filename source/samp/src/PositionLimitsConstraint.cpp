#include <lenny/samp/PositionLimitsConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

PositionLimitsConstraint::PositionLimitsConstraint(const Plan& plan) : optimization::InequalityConstraint("Position Limits"), plan(plan) {
    useTensorForHessian = false;
    barrier.setEpsilon(0.0);
    barrier.setStiffness(1.0);
}

uint PositionLimitsConstraint::getConstraintNumber() const {
    uint numActiveLimits = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++)
        if (plan.agent->getLimitsForDofIndex(i, robot::Robot::POSITION))
            numActiveLimits++;
    return plan.getNumSteps() * numActiveLimits;
}

template <typename T>
inline T sgn(T val) {
    if (val > 0)
        return (T)1;
    return (T)-1;
}

void PositionLimitsConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();  //Important!

    if (C.size() == 0)
        return;

    Eigen::VectorXd initialRobotState = plan.agent->getInitialRobotState();
    const tools::Transformation basePose = plan.agent->robot.base->getTransformationFromState(initialRobotState.segment(0, 6)) * plan.agent->localBaseTrafo.inverse();
    initialRobotState.segment(0, 6) = plan.agent->robot.base->getStateFromTransformation(basePose);
    Eigen::VectorXd agentState = plan.agent->getAgentStateFromRobotState(initialRobotState);
    const uint stateSize = agentState.size();

    uint iter = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::POSITION);
        if (!limit.has_value())
            continue;
        const bool isPositionalDof = plan.agent->isPositionalDof(i);
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index = j * stateSize + i;
            const double q_ij = q[index];
            double delta = q_ij - agentState[i];
            double lower = limit->first;
            double upper = limit->second;

            //Modification
            if (!isPositionalDof) {
                if (q_ij < 0.0)
                    delta += 2.0 * PI;
                if (fabs(delta) > PI)
                    delta -= sgn(delta) * 2.0 * PI;
                if (lower < -PI)
                    lower += 2.0 * PI;
                if (upper > PI)
                    upper -= 2.0 * PI;
            }
            agentState[i] += delta;

            //Constraints
            if (agentState[i] - limit->first < testFactor)  //lower limit is active
                C[iter] = lower - q_ij;
            else if (limit->second - q_ij < testFactor)  //upper limit is active
                C[iter] = q_ij - upper;
            iter++;
        }
    }
}

void PositionLimitsConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    Eigen::VectorXd initialRobotState = plan.agent->getInitialRobotState();
    const tools::Transformation basePose = plan.agent->robot.base->getTransformationFromState(initialRobotState.segment(0, 6)) * plan.agent->localBaseTrafo.inverse();
    initialRobotState.segment(0, 6) = plan.agent->robot.base->getStateFromTransformation(basePose);
    Eigen::VectorXd agentState = plan.agent->getAgentStateFromRobotState(initialRobotState);
    const uint stateSize = agentState.size();

    Eigen::TripletDList tripletDList;
    uint iter = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::POSITION);
        if (!limit.has_value())
            continue;
        const bool isPositionalDof = plan.agent->isPositionalDof(i);
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index = j * stateSize + i;
            const double q_ij = q[index];
            double delta = q_ij - agentState[i];
            double lower = limit->first;
            double upper = limit->second;

            //Modification
            if (!isPositionalDof) {
                if (q_ij < 0.0)
                    delta += 2.0 * PI;
                if (fabs(delta) > PI)
                    delta -= sgn(delta) * 2.0 * PI;
                if (lower < -PI)
                    lower += 2.0 * PI;
                if (upper > PI)
                    upper -= 2.0 * PI;
            }
            agentState[i] += delta;

            //Constraints
            if (agentState[i] - limit->first < testFactor)  //lower limit is active
                tools::utils::addTripletDToList(tripletDList, iter, index, -1.0);
            else if (limit->second - q_ij < testFactor)  //upper limit is active
                tools::utils::addTripletDToList(tripletDList, iter, index, 1.0);
            iter++;
        }
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
}

}  // namespace lenny::samp