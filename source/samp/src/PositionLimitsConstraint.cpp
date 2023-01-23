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
    for (const auto& [index, limit, type] : limitInfos) {
        if (type == LOWER)
            C[iter] = limit - q[index];
        else if (type == UPPER)
            C[iter] = q[index] - limit;
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
    for (const auto& [index, limit, type] : limitInfos) {
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

    //Loop over state
    for (uint i = 0; i < plan.agent->getStateSize(); i++) {
        //Get limit and check if it is active
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, robot::Robot::POSITION);
        if (!limit.has_value())
            continue;

        //Loop over trajectory
        for (uint j = 0; j < plan.getNumSteps(); j++) {
            //Index
            const uint index = j * plan.agent->getStateSize() + i;

            //lower limit is active
            if (q[index] - limit->first < testFactor) {
                limitInfos.push_back({index, limit->first, LOWER});
                if (printLimitInfos)
                    LENNY_LOG_DEBUG("Added LOWER position limit for joint '%s' at trajectory step '%d'", plan.agent->getDescriptionForDofIndex(i).c_str(), j)
            }

            //upper limit is active
            if (limit->second - q[index] < testFactor) {
                limitInfos.push_back({index, limit->second, UPPER});
                if (printLimitInfos)
                    LENNY_LOG_DEBUG("Added UPPER position limit for joint '%s' at trajectory step '%d'", plan.agent->getDescriptionForDofIndex(i).c_str(), j)
            }
        }
    }
}

}  // namespace lenny::samp