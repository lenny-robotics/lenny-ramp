#pragma once

#include <lenny/gui/Application.h>
#include <lenny/gui/Model.h>
#include <lenny/gui/Plot.h>
#include <lenny/samp/Planner.h>

namespace lenny {

class CollisionApp : public gui::Application {
public:
    CollisionApp();
    ~CollisionApp() = default;

    //--- Process
    void restart();
    void process();

    //--- Drawing
    void drawScene() const;
    void drawGui() override;
    void drawGuizmo() override;

private:
    //--- Helpers
    void setupAgentPrimitive();
    void setupWorldPrimitive();

public:
    robot::Robot robot = robot::Robot(LENNYRAMP_COLLISIONAPP_FOLDER "/config/testRobot/robot.urdf", nullptr);
    std::function<Eigen::VectorXd()> getInitialState = [&]() {
        Eigen::VectorXd initialRobotState = Eigen::VectorXd::Zero(robot.getStateSize());
        initialRobotState[0] -= 1.0;
        return initialRobotState;
    };
    rapt::Agent::SPtr agent = std::make_shared<rapt::Agent>("Agent", robot, getInitialState(), Eigen::VectorXb::Ones(robot.getStateSize()));
    rapt::WorldCollisionHandler worldCollisionHandler;
    samp::Planner planner = samp::Planner(agent, 30, 0.1, worldCollisionHandler.primitives, gui::Plot<samp::Plan::PlotType>::f_addPlot);

    enum PRIMITIVE { SPHERE, CAPSULE, RECTANGLE, BOX };
    PRIMITIVE agentBasePrimitive = SPHERE;
    PRIMITIVE agentLinkPrimitive = SPHERE;
    PRIMITIVE worldPrimitive = SPHERE;
};

}  // namespace lenny