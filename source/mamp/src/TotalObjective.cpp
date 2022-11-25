#include <lenny/mamp/AgentCollisionAvoidanceConstraint.h>
#include <lenny/mamp/TotalObjective.h>
#include <lenny/tools/Gui.h>

namespace lenny::mamp {

TotalObjective::TotalObjective(const MotionTrajectoryHandler& trajectoryHandler)
    : optimization::TotalObjective("Total MAMP Objective", 1e-5), trajectoryHandler(trajectoryHandler) {}

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

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        value += tso.computeValue(trajectory);
    }

    return value;
}

void TotalObjective::computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const {
    optimization::TotalObjective::computeGradient(pVpQ, q);

    Eigen::VectorXd trajectory, gradient;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        tso.computeGradient(gradient, trajectory);
        pVpQ.segment(startIndex, trajectory.size()) += gradient;
    }
}

void TotalObjective::computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const {
    optimization::TotalObjective::computeHessian(p2VpQ2, q);

    Eigen::VectorXd trajectory;
    uint startIndex;
    Eigen::TripletDList tripletDList;
    Eigen::SparseMatrixD hessian;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        tso.computeHessian(hessian, trajectory);
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

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        if (!tso.testIndividualFirstDerivatives(trajectory))
            testSuccessful = false;
    }

    return testSuccessful;
}

bool TotalObjective::testIndividualSecondDerivatives(const Eigen::VectorXd& q) const {
    bool testSuccessful = optimization::TotalObjective::testIndividualSecondDerivatives(q);

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        if (!tso.testIndividualSecondDerivatives(trajectory))
            testSuccessful = false;
    }

    return testSuccessful;
}

void TotalObjective::setFDCheckIsBeingApplied(bool isBeingApplied) const {
    optimization::TotalObjective::setFDCheckIsBeingApplied(isBeingApplied);
    for (const samp::TotalObjective& tso : totalSAMPObjectives)
        tso.setFDCheckIsBeingApplied(isBeingApplied);
}

void TotalObjective::preFDEvaluation(const Eigen::VectorXd& q) const {
    optimization::TotalObjective::preFDEvaluation(q);

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        tso.preFDEvaluation(trajectory);
    }
}

void TotalObjective::preValueEvaluation(const Eigen::VectorXd& q) const {
    optimization::TotalObjective::preValueEvaluation(q);

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        tso.preValueEvaluation(trajectory);
    }
}

void TotalObjective::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    optimization::TotalObjective::preDerivativeEvaluation(q);

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        tso.preDerivativeEvaluation(trajectory);
    }
}

bool TotalObjective::checkConstraintSatisfaction(const Eigen::VectorXd& q) const {
    bool checkSuccessful = optimization::TotalObjective::checkConstraintSatisfaction(q);

    Eigen::VectorXd trajectory;
    uint startIndex;
    for (const samp::TotalObjective& tso : totalSAMPObjectives) {
        trajectoryHandler.getTrajectoryForAgent(trajectory, startIndex, tso.plan.agent, q);
        if (!tso.checkConstraintSatisfaction(trajectory))
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