#include <lenny/mamp/Planner.h>
#include <lenny/tools/Gui.h>

namespace lenny::mamp {

Planner::Planner(const std::vector<PlanInfo> &planInfos, const rapt::WorldCollisionHandler::PrimitiveList &worldCollisionPrimitives,
                 tools::Plot<samp::Plan::PlotType>::F_addPlot f_addPlot)
    : trajectoryHandler(plans), objective(trajectoryHandler), optimizer("MAMP Optimizer", 1e-8), animator(planInfos.back().numSteps, planInfos.back().deltaT) {
    //Check plan infos
    if (planInfos.size() < 1)
        LENNY_LOG_ERROR("There should be at least one agent we plan for...")

    for (int i = 0; i < planInfos.size(); i++)
        for (int j = i + 1; j < planInfos.size(); j++)
            if (planInfos.at(i).agent->name == planInfos.at(j).agent->name)
                LENNY_LOG_ERROR("Agent with name `%s` seems to appear more than once...", planInfos.at(i).agent->name.c_str())

    //Setup plans
    plans.clear();
    for (const PlanInfo &info : planInfos)
        plans.emplace_back(info.agent, info.numSteps, info.deltaT, f_addPlot);

    //Setup objective (!!! Setup plans first !!!)
    objective.initialize(worldCollisionPrimitives);

    //Setup animator
    updateAnimatorParameters();
}

void Planner::resetMotionTrajectories() {
    for (samp::Plan &plan : plans)
        plan.initializeMotionTrajectory();
}

void Planner::solve(const int &maxIterations) {
    //Assemble motion trajectory
    Eigen::VectorXd stackedTrajectory;
    trajectoryHandler.assemble(stackedTrajectory);

    //Check derivatives
    if (checkIndividualDerivatives) {
        objective.testIndividualFirstDerivatives(stackedTrajectory);
        objective.testIndividualSecondDerivatives(stackedTrajectory);
    }
    if (checkTotalDerivatives) {
        objective.testGradient(stackedTrajectory);
        objective.testHessian(stackedTrajectory);
    }

    //Check constraints
    if (checkConstraints) {
        objective.checkConstraintSatisfaction(stackedTrajectory);
    }

    //Optimize
    optimizer.optimize(stackedTrajectory, objective, maxIterations);

    //Disassembly motion trajectory
    trajectoryHandler.disassemble(stackedTrajectory);

    //Update plots
    for (samp::Plan &plan : plans)
        plan.updatePlots(isRecedingHorizon);

    //Update boundary conditions
    if (isRecedingHorizon) {
        for (samp::Plan &plan : plans) {
            const Eigen::VectorXd newAgentState = plan.getAgentStateForTrajectoryIndex(0);
            plan.agent->setInitialRobotVelocityFromAgentVelocity((newAgentState - plan.agent->getInitialAgentState()) / plan.getDeltaT());
            plan.agent->setInitialRobotStateFromAgentState(newAgentState);
        }
        animator.run = false;
        animator.restart();
    }
}

void Planner::drawScene() const {
    updateAnimatorParameters();
    animator.update();
    for (const samp::Plan &plan : plans)
        plan.drawScene(animator.currentTime, isRecedingHorizon);
}

void Planner::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode("Planner")) {
        if (Gui::I->TreeNode("Plans")) {
            for (samp::Plan &plan : plans)
                plan.drawGui();
            Gui::I->TreePop();
        }

        objective.drawGui();
        optimizer.drawGui();
        animator.drawGui();

        if (Gui::I->TreeNode("Settings")) {
            Gui::I->Checkbox("Check Individual Derivatives", checkIndividualDerivatives);
            Gui::I->Checkbox("Check Total Derivatives", checkTotalDerivatives);
            Gui::I->Checkbox("Check Constraints", checkConstraints);

            Gui::I->TreePop();
        }

        Gui::I->TreePop();
    }
}

void Planner::updateAnimatorParameters() const {
    double maxAnimationTime = 0.0;
    for (const samp::Plan &plan : plans) {
        double totalAnimationTime = (double)plan.getNumSteps() * plan.getDeltaT();
        if (totalAnimationTime > maxAnimationTime) {
            animator.numSteps = plan.getNumSteps();
            animator.deltaT = plan.getDeltaT();
            maxAnimationTime = totalAnimationTime;
        }
    }
}

}  // namespace lenny::mamp