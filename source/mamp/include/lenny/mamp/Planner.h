#pragma once

#include <lenny/mamp/TotalObjective.h>
#include <lenny/optimization/NewtonOptimizer.h>
#include <lenny/samp/Plan.h>
#include <lenny/tools/Animator.h>

namespace lenny::mamp {

class Planner {
public:
    //--- Constructor
    struct PlanInfo {
        PlanInfo(rapt::Agent::SPtr agent, uint numSteps, double deltaT) : agent(agent), numSteps(numSteps), deltaT(deltaT) {}
        rapt::Agent::SPtr agent;
        uint numSteps;
        double deltaT;
    };
    Planner(const std::vector<PlanInfo> &planInfos, const rapt::WorldCollisionHandler::PrimitiveList &worldCollisionPrimitives,
            tools::Plot<samp::Plan::PlotType>::F_addPlot f_addPlot);
    ~Planner() = default;

    //--- Helpers
    void resetMotionTrajectories();

    //--- Computation
    bool solve(const int &maxIterations);

    //--- Drawing
    void drawScene() const;
    void drawGui();

private:
    //--- Helpers
    void updateAnimatorParameters() const;

public:
    //--- Members
    bool isRecedingHorizon = false;
    MotionTrajectoryHandler trajectoryHandler;
    std::vector<samp::Plan> plans;
    TotalObjective objective;
    optimization::NewtonOptimizer optimizer;

    //--- Animation
    mutable tools::Animator animator;

    //--- Booleans
    bool checkIndividualDerivatives = false;
    bool checkTotalDerivatives = false;
    bool checkConstraints = false;
};

}  // namespace lenny::mamp