#include <lenny/samp/VelocityLimitsConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

VelocityLimitsConstraint::VelocityLimitsConstraint(const Plan& plan) : optimization::InequalityConstraint("Velocity Limits"), plan(plan) {
    useTensorForHessian = false;
    barrier.setEpsilon(0.0);
    barrier.setStiffness(0.01);
}

uint VelocityLimitsConstraint::getConstraintNumber() const {
    uint numActiveLimits = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++)
        if (plan.agent->getLimitsForDofIndex(i, robot::Robot::VELOCITY))
            numActiveLimits++;
    return plan.getNumSteps() * numActiveLimits;
}

void VelocityLimitsConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();  //Important!

    if (C.size() == 0)
        return;

    const Eigen::VectorXd q_jm1 = plan.agent->getInitialAgentState();
    const uint stateSize = q_jm1.size();

    uint iter = 0;
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::VELOCITY);
        if (!limit.has_value())
            continue;

        double q_jm1i = q_jm1[i];
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index = j * stateSize + i;
            const double q_ji = q[index];
            const double vel = (q_ji - q_jm1i) / plan.getDeltaT();

            //Constraints
            if (vel - limit->first < testFactor)  //lower limit is active
                C[iter] = limit->first - vel;
            else if (limit->second - vel < testFactor)  //upper limit is active
                C[iter] = vel - limit->second;

            //Update
            q_jm1i = q_ji;
            iter++;
        }
    }
}

void VelocityLimitsConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    const Eigen::VectorXd q_jm1 = plan.agent->getInitialAgentState();
    const uint stateSize = q_jm1.size();
    const double value = 1.0 / plan.getDeltaT();

    uint iter = 0;
    Eigen::TripletDList tripletDList;
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::VELOCITY);
        if (!limit.has_value())
            continue;

        double q_jm1i = q_jm1[i];
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Preparation
            const uint index_j = j * stateSize + i;
            const int index_jm1 = (j - 1) * stateSize + i;
            const double q_ji = q[index_j];
            const double vel = (q_ji - q_jm1i) / plan.getDeltaT();

            //Constraints
            if (vel - limit->first < testFactor) {  //lower limit is active
                tools::utils::addTripletDToList(tripletDList, iter, index_j, -value);
                if (index_jm1 >= 0)
                    tools::utils::addTripletDToList(tripletDList, iter, index_jm1, value);
            } else if (limit->second - vel < testFactor) {  //upper limit is active
                tools::utils::addTripletDToList(tripletDList, iter, index_j, value);
                if (index_jm1 >= 0)
                    tools::utils::addTripletDToList(tripletDList, iter, index_jm1, -value);
            }

            //Update
            q_jm1i = q_ji;
            iter++;
        }
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void VelocityLimitsConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

void VelocityLimitsConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Test Factor", testFactor);
}

}  // namespace lenny::samp