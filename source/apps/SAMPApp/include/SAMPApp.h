#pragma once

#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>
#include <lenny/samp/Planner.h>
#include <lenny/ur_5e/Agent.h>

namespace lenny {

class SAMPApp : public gui::Application {
public:
    SAMPApp();
    ~SAMPApp() = default;

    //--- Process
    void restart();
    void process(const double& dt);

    //--- Drawing
    void drawScene() const;
    void drawGui() override;
    void drawGuizmo() override;

    //--- Interaction
    void mouseButtonCallback(double xPos, double yPos, Ray ray, int button, int action);

public:
    ur_5e::Robot robot = ur_5e::Robot(gui::Model::f_loadModel);
    ur_5e::Agent::SPtr agent = std::make_shared<ur_5e::Agent>("Agent", robot);

    rapt::WorldCollisionHandler worldCollisionHandler;
    samp::Planner planner = samp::Planner(agent, 30, 1.0 / 30.0, worldCollisionHandler.primitives, gui::Plot<samp::Plan::PlotType>::f_addPlot);

    samp::LinkTarget* selectedTarget = nullptr;
    bool useOrientationTargets = true;
    bool useMultipleTargets = false;
};

}  // namespace lenny