#include <lenny/mamp/AgentCollisionAvoidanceConstraint.h>
#include <lenny/mamp/TotalObjective.h>
#include <lenny/tools/Gui.h>

namespace lenny::mamp {

TotalObjective::TotalObjective(const MotionTrajectoryHandler& trajectoryHandler)
    : optimization::TotalObjective("Total MAMP Objective", 1e-5), trajectoryHandler(trajectoryHandler) {
    fd.f_PreEval = [&](const Eigen::VectorXd& q) -> void {
        for (const auto& [objective, weight] : subObjectives)
            if (objective->fd.f_PreEval)
                objective->fd.f_PreEval(q);

        std::unordered_map<std::string, std::pair<uint, uint>> indices;
        trajectoryHandler.getAgentTrajectoryIndices(indices);
        for (const samp::TotalObjective& tso : totalSAMPObjectives) {
            const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
            if (tso.fd.f_PreEval)
                tso.fd.f_PreEval(q.segment(startIndex, size));
        }
    };
}

void TotalObjective::initialize(const rapt::WorldCollisionHandler::PrimitiveList& worldCollisionPrimitives) {
    //Setup sub objectives
    subObjectives.clear();
    subObjectives.emplace_back(std::make_pair(std::make_unique<AgentCollisionAvoidanceConstraint>(trajectoryHandler.plans), 10.0));

    //Setup total samp objectives
    totalSAMPObjectives.clear();
    for (const samp::Plan& plan : trajectoryHandler.plans)
        totalSAMPObjectives.push_back(samp::TotalObjective(plan, worldCollisionPrimitives, "Total SAMP Objective - `" + plan.agent->name + "`", 0.0));
}

double TotalObjective::computeValue(const Eigen::VectorXd& q) const {
    double value = optimization::TotalObjective::computeValue(q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        value += tso.computeValue(q.segment(startIndex, size));
    }

    return value;
}

void TotalObjective::computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const {
    optimization::TotalObjective::computeGradient(pVpQ, q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    Eigen::VectorXd gradient;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        tso.computeGradient(gradient, q.segment(startIndex, size));
        pVpQ.segment(startIndex, size) += gradient;
    }
}

void TotalObjective::computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const {
    optimization::TotalObjective::computeHessian(p2VpQ2, q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    Eigen::TripletDList tripletDList;
    Eigen::SparseMatrixD hessian;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        tso.computeHessian(hessian, q.segment(startIndex, size));
        for (int k = 0; k < hessian.outerSize(); ++k)
            for (Eigen::SparseMatrixD::InnerIterator it(hessian, k); it; ++it)
                tools::utils::addTripletDToList(tripletDList, startIndex + it.row(), startIndex + it.col(), it.value());
    }
    hessian.resize(q.size(), q.size());
    hessian.setFromTriplets(tripletDList.begin(), tripletDList.end());
    p2VpQ2 += hessian;
}

bool TotalObjective::testIndividualFirstDerivatives(const Eigen::VectorXd& q) const {
    bool testSuccessful = optimization::TotalObjective::testIndividualFirstDerivatives(q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        if (!tso.testIndividualFirstDerivatives(q.segment(startIndex, size)))
            testSuccessful = false;
    }

    return testSuccessful;
}

bool TotalObjective::testIndividualSecondDerivatives(const Eigen::VectorXd& q) const {
    bool testSuccessful = optimization::TotalObjective::testIndividualSecondDerivatives(q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        if (!tso.testIndividualSecondDerivatives(q.segment(startIndex, size)))
            testSuccessful = false;
    }

    return testSuccessful;
}

bool TotalObjective::testGradient(const Eigen::VectorXd& x) const {
    for (const samp::TotalObjective& tso : totalSAMPObjectives)
        tso.fdCheckIsBeingApplied = true;
    const bool successful = optimization::TotalObjective::testGradient(x);
    for (const samp::TotalObjective& tso : totalSAMPObjectives)
        tso.fdCheckIsBeingApplied = false;
    return successful;
}

bool TotalObjective::testHessian(const Eigen::VectorXd& x) const {
    for (const samp::TotalObjective& tso : totalSAMPObjectives)
        tso.fdCheckIsBeingApplied = true;
    const bool successful = optimization::TotalObjective::testHessian(x);
    for (const samp::TotalObjective& tso : totalSAMPObjectives)
        tso.fdCheckIsBeingApplied = false;
    return successful;
}

bool TotalObjective::preValueEvaluation(const Eigen::VectorXd& q) const {
    bool successful = optimization::TotalObjective::preValueEvaluation(q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        if (!tso.preValueEvaluation(q.segment(startIndex, size)))
            successful = false;
    }

    return successful;
}

void TotalObjective::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    optimization::TotalObjective::preDerivativeEvaluation(q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        tso.preDerivativeEvaluation(q.segment(startIndex, size));
    }

    for (const auto& [objective, weight] : subObjectives)
        if (const optimization::Constraint* con = dynamic_cast<optimization::Constraint*>(objective.get()))
            con->softificationWeights.setOnes(con->getConstraintNumber());
}

bool TotalObjective::checkConstraintSatisfaction(const Eigen::VectorXd& q) const {
    bool checkSuccessful = optimization::TotalObjective::checkConstraintSatisfaction(q);

    std::unordered_map<std::string, std::pair<uint, uint>> indices;
    trajectoryHandler.getAgentTrajectoryIndices(indices);
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        const auto& [startIndex, size] = indices.at(tso.plan.agent->name);
        if (!tso.checkConstraintSatisfaction(q.segment(startIndex, size)))
            checkSuccessful = false;
    }

    return checkSuccessful;
}

void TotalObjective::drawGui() {
    if (tools::Gui::I->TreeNode("Objectives")) {
        optimization::TotalObjective::drawGui();
        for (samp::TotalObjective& tso : totalSAMPObjectives)
            tso.drawGui();
        tools::Gui::I->TreePop();
    }
}

}  // namespace lenny::mamp