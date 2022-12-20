#pragma once

#include <lenny/optimization/NewtonOptimizer.h>
#include <lenny/samp/Plan.h>
#include <lenny/samp/TotalObjective.h>
#include <lenny/tools/Animator.h>

namespace lenny::samp {

class Planner {
public:
    //--- Constructor
    Planner(const rapt::Agent::SPtr agent, const uint& numSteps, const double& deltaT,
            const rapt::WorldCollisionHandler::PrimitiveList& worldCollisionPrimitives, tools::Plot<Plan::PlotType>::F_addPlot f_addPlot);
    ~Planner() = default;

    //--- Computation
    void solve(const int& maxIterations);

    //--- Drawing
    void drawScene() const;
    void drawGui();

public:
    //--- Members
    bool isRecedingHorizon = false;
    Plan plan;
    TotalObjective objective;
    optimization::NewtonOptimizer optimizer;

    //--- Animation
    mutable tools::Animator animator;

    //--- Booleans
    bool checkIndividualDerivatives = false;
    bool checkTotalDerivatives = false;
    bool checkConstraints = false;
};

}  // namespace lenny::samp