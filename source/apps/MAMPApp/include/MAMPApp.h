#pragma once

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
    std::array<robot::Robot, 1> robots = {robot::Robot(LENNY_ROBOT_FOLDER "/data/floating_base/robot.urdf", gui::Model::f_loadModel)};
    std::array<rapt::Agent::SPtr, 2> agents = {
        std::make_shared<rapt::Agent>("Floating Base 1", robots.back(), Eigen::VectorXd::Zero(robots.back().getStateSize()),
                                      Eigen::VectorXb::Ones(robots.back().getStateSize())),
        std::make_shared<rapt::Agent>("Floating Base 2", robots.back(), Eigen::VectorXd::Zero(robots.back().getStateSize()),
                                      Eigen::VectorXb::Ones(robots.back().getStateSize()))};
    rapt::WorldCollisionHandler worldCollisionHandler;
    mamp::Planner planner = mamp::Planner({{agents.at(0), 30, 1.0 / targetFramerate}, {agents.at(1), 30, 1.0 / targetFramerate}},
                                          worldCollisionHandler.primitives, gui::Plot<samp::Plan::PlotType>::f_addPlot);

    samp::LinkTarget* selectedTarget = nullptr;
    bool useMultipleTargets = false;
    bool useOrientationTargets = false;
};

}  // namespace lenny