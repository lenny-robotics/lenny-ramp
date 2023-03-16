#include "MAMPApp.h"

#include <lenny/gui/Guizmo.h>
#include <lenny/gui/ImGui.h>

namespace lenny {

MAMPApp::MAMPApp() : gui::Application("MAMPApp") {
    //Setup process
    processes.emplace_back(std::make_shared<gui::Process>(
        "Process-1", [&]() -> void { process(1.0 / targetFramerate); }, [&]() -> void { restart(); }));

    //Setup scene
    const auto [width, height] = getCurrentWindowSize();
    scenes.emplace_back(std::make_shared<gui::Scene>("Scene-1", width, height));
    scenes.back()->f_drawScene = [&]() -> void { drawScene(); };
    scenes.back()->f_mouseButtonCallback = [&](double xPos, double yPos, Ray ray, int button, int action) -> void {
        mouseButtonCallback(xPos, yPos, ray, button, action);
    };

    //Reset initial state
    Eigen::VectorXd initialRobotState = agents.at(1)->getInitialRobotState();
    initialRobotState[0] = 2.0;
    initialRobotState[5] = PI;
    agents.at(1)->setInitialRobotStateFromRobotState(initialRobotState);
    planner.resetMotionTrajectories();

    //Set animator time
    planner.animator.setCurrentTimeFromPercentage(1.0);

    //Add collision primitives
    for (rapt::Agent::SPtr agent : agents) {
        agent->collisionPrimitives.clear();
        agent->addCollisionSphere("body", Eigen::Vector3d(0.0, -0.2, 0.0), 0.6);
    }
}

void MAMPApp::restart() {
    planner.resetMotionTrajectories();
}

void MAMPApp::process(const double& dt) {
    if (planner.isRecedingHorizon) {
        for (samp::Plan& plan : planner.plans)
            plan.setDeltaT(dt);
    }
    planner.solve(1);
}

void MAMPApp::drawScene() const {
    planner.drawScene();
    worldCollisionHandler.drawScene();
}

void MAMPApp::drawGui() {
    ImGui::Begin("Main Menu");

    if (ImGui::TreeNode("App Settings")) {
        ImGui::Checkbox("Use Multiple Targets", &useMultipleTargets);
        ImGui::Checkbox("Use orientation targets", &useOrientationTargets);
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Robots")) {
        spotFloatingRobot.drawGui(false);
        spotBaseRobot.drawGui(false);

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Agents")) {
        for (rapt::Agent::SPtr& agent : agents)
            agent->drawGui(true);
        ImGui::TreePop();
    }

    planner.drawGui();
    worldCollisionHandler.drawGui();

    ImGui::End();
}

void MAMPApp::drawGuizmo() {
    if (selectedTarget) {
        static Eigen::Vector3d scale = Eigen::Vector3d::Ones();
        static Eigen::QuaternionD orientation = Eigen::QuaternionD::Identity();
        gui::Guizmo::useWidget(selectedTarget->position->global, selectedTarget->orientation.has_value() ? selectedTarget->orientation->global : orientation,
                               scale);
    }
}

void MAMPApp::mouseButtonCallback(double xPos, double yPos, Ray ray, int button, int action) {
    if (action == GLFW_PRESS) {
        selectedTarget = nullptr;

        //Right mouse button
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            for (samp::Plan& plan : planner.plans)
                if (plan.linkTargets.size() > 0)
                    plan.linkTargets.pop_back();
        }

        //Left mouse button
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            for (samp::Plan& plan : planner.plans) {
                std::vector<samp::LinkTarget>& linkTargets = plan.linkTargets;
                const robot::Robot& robot = plan.agent->robot;
                const Eigen::VectorXd robotState = plan.agent->getRobotStateFromAgentState(plan.getAgentStateForTrajectoryTime(planner.animator.currentTime));
                const auto firstLink = robot.getFirstLinkHitByRay(robotState, ray);
                if (firstLink.has_value()) {
                    if (!useMultipleTargets)
                        linkTargets.clear();

                    const auto& [linkName, selectedPoint] = firstLink.value();

                    const samp::LinkTarget::Position positionTarget(selectedPoint, robot.computeLocalPoint(robotState, selectedPoint, linkName),
                                                                    Eigen::Vector3d::Ones());
                    if (useOrientationTargets) {
                        const samp::LinkTarget::Orientation orientationTarget(
                            robot.computeGlobalOrientation(robotState, Eigen::QuaternionD::Identity(), linkName), Eigen::QuaternionD::Identity(),
                            Eigen::Vector3d::Ones());

                        linkTargets.emplace_back(plan.agent, linkName, 1.0, positionTarget, orientationTarget);
                    } else {
                        linkTargets.emplace_back(plan.agent, linkName, 1.0, positionTarget);
                    }

                    selectedTarget = &linkTargets.back();
                    break;
                }
            }
        }
    }
}

}  // namespace lenny