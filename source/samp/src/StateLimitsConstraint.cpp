#include <lenny/samp/StateLimitsConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

StateLimitsConstraint::StateLimitsConstraint(const std::string& description, const Plan& plan, const robot::Robot::LIMITS_TYPE& limitsType,
                                             const double& stiffness, const double& testFactor)
    : optimization::InequalityConstraint(description), plan(plan), limitsType(limitsType), testFactor(testFactor) {
    useTensorForHessian = false;
    barrier.setStiffness(stiffness);
    barrier.setEpsilon(0.0);
}

uint StateLimitsConstraint::getConstraintNumber() const {
    return limitInfos.size();
}

void StateLimitsConstraint::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();

    if (C.size() == 0)
        return;

    uint iter = 0;
    for (const auto& [index, limit, type] : limitInfos) {
        const double value = computeValue(q, index);
        if (type == LOWER)
            C[iter] = limit - value;
        else
            C[iter] = value - limit;
        iter++;
    }
}

void StateLimitsConstraint::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    Eigen::TripletDList tripletDList;
    uint iter = 0;
    for (const auto& [index, limit, type] : limitInfos) {
        const double sign = (type == LOWER) ? -1.0 : 1.0;
        const auto entries = computeValueDerivative(q, index);
        for (const auto& [val, ind] : entries)
            tools::utils::addTripletDToList(tripletDList, iter, ind, sign * val);
        iter++;
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

void StateLimitsConstraint::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();
}

void StateLimitsConstraint::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Test Factor", testFactor);
    tools::Gui::I->Checkbox("Print Limit Infos", printLimitInfos);
}

void StateLimitsConstraint::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    setupLimitInfos(q);
}

void StateLimitsConstraint::setupLimitInfos(const Eigen::VectorXd& q) const {
    //Clear list
    limitInfos.clear();

    //Loop over state
    const int stateSize = plan.agent->getStateSize();
    for (int i = 0; i < (int)plan.agent->getStateSize(); i++) {
        //Get limit and check if it is active
        const robot::Limits& limit = plan.agent->getLimitsForDofIndex(i, limitsType);
        if (!limit.has_value())
            continue;

        //Loop over trajectory
        for (int j = 0; j < (int)plan.getNumSteps(); j++) {
            //Value
            const int index = j * stateSize + i;
            const double value = computeValue(q, index);

            //lower limit is active
            if (value - limit->first < testFactor) {
                limitInfos.push_back({index, limit->first, LOWER});
                if (printLimitInfos)
                    LENNY_LOG_DEBUG("Added LOWER limit for DOF '%s' at trajectory step '%d'", plan.agent->getDescriptionForDofIndex(i).c_str(), j)
            }

            //upper limit is active
            if (limit->second - value < testFactor) {
                limitInfos.push_back({index, limit->second, UPPER});
                if (printLimitInfos)
                    LENNY_LOG_DEBUG("Added UPPER limit for DOF '%s' at trajectory step '%d'", plan.agent->getDescriptionForDofIndex(i).c_str(), j)
            }
        }
    }
}

}  // namespace lenny::samp