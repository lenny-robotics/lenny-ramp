#pragma once

#include <lenny/bd_spot/Agent.h>
#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>
#include <lenny/mamp/Planner.h>

namespace lenny {

class MAMPApp : public gui::Application {
public:
    MAMPApp();
    ~MAMPApp() = default;

    //--- Process
    void restart() override;
    void process() override;

    //--- Drawing
    void drawScene() const override;
    void drawGui() override;

    //--- Interaction
    void mouseButtonCallback(double xPos, double yPos, int button, int action) override;

public:
    bd_spot::FloatingRobot spotFloatingRobot = bd_spot::FloatingRobot(gui::Model::f_loadModel);
    bd_spot::BaseRobot spotBaseRobot = bd_spot::BaseRobot(gui::Model::f_loadModel);

    std::array<rapt::Agent::SPtr, 2> agents = {std::make_shared<bd_spot::BaseAgent>("Spot Base 1", spotFloatingRobot, spotBaseRobot),
                                               std::make_shared<bd_spot::BaseAgent>("Spot Base 2", spotFloatingRobot, spotBaseRobot)};
    rapt::WorldCollisionHandler worldCollisionHandler;
    mamp::Planner planner = mamp::Planner({{agents.at(0), 60, 1.0 / 30.0}, {agents.at(1), 60, 1.0 / 30.0}}, worldCollisionHandler.primitives,
                                          gui::Plot<samp::Plan::PlotType>::f_addPlot);

    samp::LinkTarget* selectedTarget = nullptr;
    bool useMultipleTargets = false;
    bool useOrientationTargets = false;
};

}  // namespace lenny