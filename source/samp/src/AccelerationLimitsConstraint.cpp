#include <lenny/samp/AccelerationLimitsConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

AccelerationLimitsConstraint::AccelerationLimitsConstraint(const Plan& plan) : optimization::InequalityConstraint("Acceleration Limits"), plan(plan) {
    useTensorForHessian = false;
    barrier.setEpsilon(0.0);
    barrier.setStiffness(0.01);  //ToDo: Is this a good value?
}

uint AccelerationLimitsConstraint::getConstraintNumber() const {
    uint numActiveLimits = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++)
        if (plan.agent->getLimitsForDofIndex(i, robot::Robot::ACCELERATION))
            numActiveLimits++;
    return plan.getNumSteps() * numActiveLimits;
}

void AccelerationLimitsConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();  //Important!

    if (C.size() == 0)
        return;

    const Eigen::VectorXd q_jm2 = plan.agent->getInitialAgentState() - plan.getDeltaT() * plan.agent->getInitialAgentVelocity();
    const Eigen::VectorXd q_jm1 = plan.agent->getInitialAgentState();
    const uint stateSize = q_jm1.size();

    uint iter = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::ACCELERATION);
        if (!limit.has_value())
            continue;

        double q_jm2i = q_jm2[i];
        double q_jm1i = q_jm1[i];
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index = j * stateSize + i;
            const double q_ji = q[index];
            const double acc = (q_ji - 2.0 * q_jm1i + q_jm2i) / (plan.getDeltaT() * plan.getDeltaT());

            //Constraints
            if (acc - limit->first < testFactor)  //lower limit is active
                C[iter] = limit->first - acc;
            else if (limit->second - acc < testFactor)  //upper limit is active
                C[iter] = acc - limit->second;

            //Update
            q_jm2i = q_jm1i;
            q_jm1i = q_ji;
            iter++;
        }
    }
}

void AccelerationLimitsConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    const Eigen::VectorXd q_jm2 = plan.agent->getInitialAgentState() - plan.getDeltaT() * plan.agent->getInitialAgentVelocity();
    const Eigen::VectorXd q_jm1 = plan.agent->getInitialAgentState();
    const uint stateSize = q_jm1.size();
    const double value = 1.0 / (plan.getDeltaT() * plan.getDeltaT());

    uint iter = 0;
    Eigen::TripletDList tripletDList;
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::ACCELERATION);
        if (!limit.has_value())
            continue;

        double q_jm2i = q_jm2[i];
        double q_jm1i = q_jm1[i];
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index_j = j * stateSize + i;
            const uint index_jm1 = (j - 1) * stateSize + i;
            const uint index_jm2 = (j - 2) * stateSize + i;
            const double q_ji = q[index_j];
            const double acc = (q_ji - 2.0 * q_jm1i + q_jm2i) / (plan.getDeltaT() * plan.getDeltaT());

            //Constraints
            double sign = 0.0;
            if (acc - limit->first < testFactor)  //lower limit is active
                sign = -1.0;
            else if (limit->second - acc < testFactor)  //upper limit is active
                sign = 1.0;
            tools::utils::addTripletDToList(tripletDList, iter, index_j, sign * value);
            if (index_jm1 >= 0)
                tools::utils::addTripletDToList(tripletDList, iter, index_jm1, -sign * 2.0 * value);
            if (index_jm2 >= 0)
                tools::utils::addTripletDToList(tripletDList, iter, index_jm2, sign * value);

            //Update
            q_jm2i = q_jm1i;
            q_jm1i = q_ji;
            iter++;
        }
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void AccelerationLimitsConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

void AccelerationLimitsConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Test Factor", testFactor);
}

}  // namespace lenny::samp