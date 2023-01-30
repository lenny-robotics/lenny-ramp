#include <lenny/samp/LinkLimitsConstraint.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

template <typename T>
LinkLimitsConstraint<T>::LinkLimitsConstraint(const std::string& description, const Plan& plan, const double& stiffness, const double& testFactor)
    : optimization::InequalityConstraint(description), plan(plan), testFactor(testFactor) {
    useTensorForHessian = false;
    barrier.setStiffness(stiffness);
    barrier.setEpsilon(0.0);
}

template <typename T>
uint LinkLimitsConstraint<T>::getConstraintNumber() const {
    return 3 * limitInfos.size();
}

template <typename T>
void LinkLimitsConstraint<T>::computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const {
    C.resize(getConstraintNumber());
    C.setZero();

    if (C.size() == 0)
        return;

    uint iter = 0;
    for (const auto& [limits, linkName, boundType, index] : limitInfos) {
        const Eigen::Vector3d value = computeValue(q, linkName, limits.local, index);
        if (boundType == LOWER)
            C.segment(iter, 3) = limits.lower - value;
        else if (boundType == UPPER)
            C.segment(iter, 3) = value - limits.upper;
        iter += 3;
    }
}

template <typename T>
void LinkLimitsConstraint<T>::computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const {
    pCpQ.resize(getConstraintNumber(), q.size());
    pCpQ.setZero();

    if (pCpQ.rows() == 0)
        return;

    Eigen::TripletDList tripletDList;
    uint iter = 0;
    for (const auto& [limits, linkName, boundType, index] : limitInfos) {
        const double sign = (boundType == LOWER) ? -1.0 : 1.0;
        Eigen::TripletDList jacobian;
        computeJacobian(jacobian, q, linkName, limits.local, index);
        for (const auto& entry : jacobian)
            tools::utils::addTripletDToList(tripletDList, iter + entry.row(), index + entry.col(), sign * entry.value());
        iter += 3;
    }
    pCpQ.setFromTriplets(tripletDList.begin(), tripletDList.end());
}

template <typename T>
void LinkLimitsConstraint<T>::computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const {
    p2CpQ2.resize(Eigen::Vector3i(getConstraintNumber(), q.size(), q.size()));
    p2CpQ2.setZero();

    //ToDo: Add more!
}

template <typename T>
void LinkLimitsConstraint<T>::drawGuiContent() {
    optimization::InequalityConstraint::drawGuiContent();
    tools::Gui::I->Input("Test Factor", testFactor);
    tools::Gui::I->Checkbox("Print Limit Infos", printLimitInfos);
}

template <>
void LinkLimitsConstraint<Eigen::Vector3d>::setupLimitInfos(const Eigen::VectorXd& q, const std::vector<LinkLimits>& linkLimitsList, const double& dt) const {
    //Clear list
    limitInfos.clear();

    //Loop over list
    for (const LinkLimits& linkLimits : linkLimitsList) {
        //Check if limits are active
        if (!linkLimits.linear.has_value())
            continue;

        //Loop over index range
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(linkLimits.range.get().first),
                                                  plan.getTrajectoryIndexForPercentage(linkLimits.range.get().second)};
        for (int i = (int)indexRange.first; i <= (int)indexRange.second; i++) {
            //Get value
            const Eigen::Vector3d value = computeValue(q, linkLimits.linkName, linkLimits.linear->local, i);

            //lower limit is active
            for (int j = 0; j < 3; j++) {
                if ((value - linkLimits.linear->lower)[j] < testFactor / dt) {
                    limitInfos.push_back({linkLimits.linear.value(), linkLimits.linkName, LOWER, i});
                    if (printLimitInfos)
                        LENNY_LOG_DEBUG("Added LINEAR LOWER limit for link '%s' at trajectory step '%d'", linkLimits.linkName.c_str(), i)
                    break;
                }
            }

            //upper limit is active
            for (int j = 0; j < 3; j++) {
                if ((linkLimits.linear->upper - value)[j] < testFactor / dt) {
                    limitInfos.push_back({linkLimits.linear.value(), linkLimits.linkName, UPPER, i});
                    if (printLimitInfos)
                        LENNY_LOG_DEBUG("Added LINEAR UPPER limit for link '%s' at trajectory step '%d'", linkLimits.linkName.c_str(), i)
                    break;
                }
            }
        }
    }
}

template <>
void LinkLimitsConstraint<Eigen::QuaternionD>::setupLimitInfos(const Eigen::VectorXd& q, const std::vector<LinkLimits>& linkLimitsList, const double& dt) const {
    //Clear list
    limitInfos.clear();

    //Loop over list
    for (const LinkLimits& linkLimits : linkLimitsList) {
        //Check if limits are active
        if (!linkLimits.angular.has_value())
            continue;

        //Loop over index range
        const std::pair<uint, uint> indexRange = {plan.getTrajectoryIndexForPercentage(linkLimits.range.get().first),
                                                  plan.getTrajectoryIndexForPercentage(linkLimits.range.get().second)};
        for (int i = (int)indexRange.first; i <= (int)indexRange.second; i++) {
            //Get value
            const Eigen::Vector3d value = computeValue(q, linkLimits.linkName, linkLimits.angular->local, i);

            //lower limit is active
            for (int j = 0; j < 3; j++) {
                if ((value - linkLimits.angular->lower)[j] < testFactor / dt) {
                    limitInfos.push_back({linkLimits.angular.value(), linkLimits.linkName, LOWER, i});
                    if (printLimitInfos)
                        LENNY_LOG_DEBUG("Added ANGULAR LOWER limit for link '%s' at trajectory step '%d'", linkLimits.linkName.c_str(), i)
                    break;
                }
            }

            //upper limit is active
            for (int j = 0; j < 3; j++) {
                if ((linkLimits.angular->upper - value)[j] < testFactor / dt) {
                    limitInfos.push_back({linkLimits.angular.value(), linkLimits.linkName, UPPER, i});
                    if (printLimitInfos)
                        LENNY_LOG_DEBUG("Added ANGULAR UPPER limit for link '%s' at trajectory step '%d'", linkLimits.linkName.c_str(), i)
                    break;
                }
            }
        }
    }
}

template class LinkLimitsConstraint<Eigen::Vector3d>;
template class LinkLimitsConstraint<Eigen::QuaternionD>;

}  // namespace lenny::samp