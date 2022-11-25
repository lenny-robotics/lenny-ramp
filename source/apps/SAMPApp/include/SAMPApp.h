#pragma once

#include <lenny/agents/ABBYuMiAgent.h>
#include <lenny/agents/BDSpotArmAgent.h>
#include <lenny/agents/BDSpotBaseAgent.h>
#include <lenny/agents/FloatingBaseAgent.h>
#include <lenny/agents/FrankaPandaAgent.h>
#include <lenny/agents/KinovaGen3Agent.h>
#include <lenny/agents/UR5eAgent.h>
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
    //    agents::ABBYuMiRobot robot = agents::ABBYuMiRobot(gui::Model::f_loadModel);
    //    rapt::Agent::SPtr agent = std::make_shared<agents::ABBYuMiAgent>("Agent", robot);

    //    agents::BDSpotArmRobot robot = agents::BDSpotArmRobot(gui::Model::f_loadModel);
    //    agents::BDSpotBaseRobot spotRobot = agents::BDSpotBaseRobot(gui::Model::f_loadModel);
    //    rapt::Agent::SPtr agent = std::make_shared<agents::BDSpotArmAgent>("Agent", robot, spotRobot);

    //    agents::BDSpotFloatingRobot robot = agents::BDSpotFloatingRobot(gui::Model::f_loadModel);
    //    agents::BDSpotBaseRobot spotRobot = agents::BDSpotBaseRobot(gui::Model::f_loadModel);
    //    rapt::Agent::SPtr agent = std::make_shared<agents::BDSpotBaseAgent>("Agent", robot, spotRobot);

    //    agents::FloatingBaseRobot robot = agents::FloatingBaseRobot(gui::Model::f_loadModel);
    //    rapt::Agent::SPtr agent = std::make_shared<agents::FloatingBaseAgent>("Agent", robot);

    //    agents::FrankaPandaRobot robot = agents::FrankaPandaRobot(gui::Model::f_loadModel);
    //    rapt::Agent::SPtr agent = std::make_shared<agents::FrankaPandaAgent>("Agent", robot);

    //    agents::KinovaGen3Robot robot = agents::KinovaGen3Robot(gui::Model::f_loadModel);
    //    rapt::Agent::SPtr agent = std::make_shared<agents::KinovaGen3Agent>("Agent", robot);

    agents::UR5eRobot robot = agents::UR5eRobot(gui::Model::f_loadModel);
    rapt::Agent::SPtr agent = std::make_shared<agents::UR5eAgent>("Agent", robot);

    rapt::WorldCollisionHandler worldCollisionHandler;
    samp::Planner planner = samp::Planner(agent, 60, 1.0 / targetFramerate, worldCollisionHandler.primitives, gui::Plot<samp::Plan::PlotType>::f_addPlot);

    samp::LinkTarget* selectedTarget = nullptr;
    bool useOrientationTargets = true;
    bool useMultipleTargets = false;
};

}  // namespace lenny