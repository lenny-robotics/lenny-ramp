#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/LinkLimits.h>
#include <lenny/samp/LinkRegularizer.h>
#include <lenny/samp/LinkTarget.h>
#include <lenny/samp/StateRegularizer.h>
#include <lenny/samp/StateTarget.h>
#include <lenny/tools/Plot.h>

namespace lenny::samp {

class Plan {
public:
    //--- Typedefs
    typedef std::array<double, 3> PlotType;  //[lower limit, upper limit, value]

    //--- Constructor
    Plan(const rapt::Agent::SPtr agent, const uint& numSteps, const double& deltaT, tools::Plot<PlotType>::F_addPlot f_addPlot);
    Plan(Plan&& other);
    ~Plan() = default;

    //--- Trajectory helpers
    void initializeMotionTrajectory();

    Eigen::VectorXd getAgentStateForTrajectoryIndex(const int& index) const;
    Eigen::VectorXd getAgentStateForTrajectoryTime(const double& time) const;

    int getTrajectoryIndexForTime(const double& time) const;
    double getTrajectoryTimeForIndex(const int& index) const;
    double getTotalTrajectoryTime() const;

    //--- Objective helpers
    Eigen::VectorXd getAgentStateForTrajectoryIndex(const Eigen::VectorXd& motionTrajectory, const int& index) const;
    Eigen::VectorXd getAgentStateForTrajectoryTime(const Eigen::VectorXd& motionTrajectory, const double& time) const;
    Eigen::VectorXd getAgentStateForTrajectoryPercentage(const Eigen::VectorXd& motionTrajectory, const double& percentage) const;
    uint getTrajectoryIndexForPercentage(const double& percentage) const;

    //--- Setter
    void setNumSteps(const uint& numSteps);
    void setDeltaT(const double& deltaT);
    void setDofMask(const Eigen::VectorXb& dofMask);

    //--- Getter
    const uint& getNumSteps() const;
    const double& getDeltaT() const;

    //--- Plot helpers
    void initializePlots();
    void updatePlots(const bool& isRecedingHorizon);

    //--- Drawing
    void drawGui(const bool& withTrajectoryStettings = true);
    void drawScene(const double& currentAnimationTime, const bool& isRecedingHorizon) const;

private:
    //--- Drawing
    void drawEndEffectorTrajectories() const;
    void drawSkeletonTrajectory() const;
    void drawVisualsTrajectory() const;
    void drawLinkTargets() const;
    void drawStateTargets() const;

public:
    //--- Public members
    const rapt::Agent::SPtr agent;
    Eigen::VectorXd motionTrajectory;

    //--- Lists
    std::vector<LinkTarget> linkTargets;
    std::vector<LinkLimits> linkLimits;  //ToDo: Add more schtuff
    std::vector<LinkRegularizer> linkRegularizers;
    std::vector<StateTarget> stateTargets;
    std::vector<StateRegularizer> stateRegularizers;

    //--- Drawing
    mutable bool showEndEffectorTrajectories = true;
    mutable bool showSkeletonTrajectory = true;
    mutable bool showVisualsTrajectory = false;
    mutable bool showLinkTargets = true;
    mutable bool showStateTargets = true;

    mutable uint trajectoryDrawingInterval = 1;

protected:
    //--- Protected members (set by constructor)
    uint numSteps;
    double deltaT;

    //--- Plots
    tools::Plot<PlotType>::F_addPlot f_addPlot;  //Set by constructor
    tools::Plot<PlotType>::List positionPlots, velocityPlots;
};

}  // namespace lenny::samp
