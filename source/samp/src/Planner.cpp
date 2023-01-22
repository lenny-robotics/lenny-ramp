#include <lenny/samp/Planner.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

Planner::Planner(const rapt::Agent::SPtr agent, const uint& numSteps, const double& deltaT,
                 const rapt::WorldCollisionHandler::PrimitiveList& worldCollisionPrimitives, tools::Plot<Plan::PlotType>::F_addPlot f_addPlot)
    : plan(agent, numSteps, deltaT, f_addPlot),
      objective(plan, worldCollisionPrimitives),
      optimizer("SAMP Optimizer", 1e-8),
      animator(plan.getNumSteps(), plan.getDeltaT()) {
    optimizer.printInfos = true;
}

bool Planner::solve(const int& maxIterations) {
    //Check derivatives
    if (checkIndividualDerivatives) {
        objective.testIndividualFirstDerivatives(plan.motionTrajectory);
        objective.testIndividualSecondDerivatives(plan.motionTrajectory);
    }
    if (checkTotalDerivatives) {
        objective.testGradient(plan.motionTrajectory);
        objective.testHessian(plan.motionTrajectory);
    }

    //Check constraints
    if (checkConstraints) {
        objective.checkConstraintSatisfaction(plan.motionTrajectory);
    }

    //Optimize
    const bool converged = optimizer.optimize(plan.motionTrajectory, objective, maxIterations);

    //Update plots
    plan.updatePlots(isRecedingHorizon);

    //Update boundary conditions
    if (isRecedingHorizon) {
        const Eigen::VectorXd newAgentState = plan.getAgentStateForTrajectoryIndex(0);
        plan.agent->setInitialRobotVelocityFromAgentVelocity((newAgentState - plan.agent->getInitialAgentState()) /
                                                             plan.getDeltaT());  //ToDo: is PI VS -PI a problem here?
        plan.agent->setInitialRobotStateFromAgentState(newAgentState);
        animator.run = false;
        animator.restart();
    }

    return converged;
}

void Planner::drawScene() const {
    animator.update();
    plan.drawScene(animator.currentTime, isRecedingHorizon);
}

void Planner::drawGui() {
    using tools::Gui;
    if (Gui::I->TreeNode(("Planner - `" + plan.agent->name + "`").c_str())) {
        plan.drawGui();
        objective.drawGui();
        optimizer.drawGui();
        animator.drawGui();

        if (Gui::I->TreeNode("Settings")) {
            Gui::I->Checkbox("Is Receding Horizon", isRecedingHorizon);

            Gui::I->Checkbox("Check Individual Derivatives", checkIndividualDerivatives);
            Gui::I->Checkbox("Check Total Derivatives", checkTotalDerivatives);
            Gui::I->Checkbox("Check Constraints", checkConstraints);

            Gui::I->TreePop();
        }

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp