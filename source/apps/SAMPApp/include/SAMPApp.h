#pragma once

#include <lenny/bd_spot/Agent.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>
#include <lenny/samp/Planner.h>

namespace lenny {

class SAMPApp : public gui::Application {
public:
    SAMPApp();
    ~SAMPApp() = default;

    //--- Process
    void restart() override;
    void process() override;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

    //--- Interaction
    void mouseButtonCallback(double xPos, double yPos, int button, int action) override;

public:
    bd_spot::ArmRobot robot = bd_spot::ArmRobot(gui::Model::f_loadModel);
    bd_spot::BaseRobot baseRobot = bd_spot::BaseRobot(gui::Model::f_loadModel);
    rapt::Agent::SPtr agent = std::make_shared<bd_spot::ArmAgent>("Agent", robot, baseRobot);

    rapt::WorldCollisionHandler worldCollisionHandler;
    samp::Planner planner = samp::Planner(agent, 1, 1.0 / targetFramerate, worldCollisionHandler.primitives, gui::Plot<samp::Plan::PlotType>::f_addPlot);

    samp::LinkTarget* selectedTarget = nullptr;
    bool useOrientationTargets = true;
    bool useMultipleTargets = false;
};

}  // namespace lenny