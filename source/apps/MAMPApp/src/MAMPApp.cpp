#include "MAMPApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/ImGuizmo.h>

namespace lenny {

MAMPApp::MAMPApp() : gui::Application("MAMPApp") {
    //Setup drawing
    showOrigin = false;
    showGround = false;

    //Reset initial states
    for (rapt::Agent::SPtr agent : agents) {
        Eigen::VectorXd initialRobotState = agent->getInitialRobotState();
        initialRobotState[0] = tools::utils::getRandomNumberInRange({-1.0, 1.0});
        initialRobotState[2] = tools::utils::getRandomNumberInRange({-1.0, 1.0});
        agent->setInitialRobotStateFromRobotState(initialRobotState);
    }
    planner.resetMotionTrajectories();

    //Set animator time
    planner.animator.setCurrentTimeFromPercentage(1.0);

    //Add collision primitives
    for (auto& agent : agents) {
        agent->collisionPrimitives.clear();
        agent->addCollisionSphere("base", Eigen::Vector3d::Zero(), 0.2);
    }
}

void MAMPApp::restart() {
    planner.resetMotionTrajectories();
}

void MAMPApp::process() {
    if (planner.isRecedingHorizon) {
        for (samp::Plan& plan : planner.plans)
            plan.setDeltaT(1.0 / targetFramerate);
    }
    planner.solve(1);
}

void MAMPApp::drawScene() const {
    planner.drawScene();
    worldCollisionHandler.drawScene();
}

void MAMPApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    if (ImGui::TreeNode("App Settings")) {
        ImGui::Checkbox("Use Multiple Targets", &useMultipleTargets);
        ImGui::Checkbox("Use orientation targets", &useOrientationTargets);
        ImGui::TreePop();
    }

    for (robot::Robot& robot : robots)
        robot.drawGui(false);
    for (rapt::Agent::SPtr& agent : agents)
        agent->drawGui(true);
    planner.drawGui();
    worldCollisionHandler.drawGui();

    if (selectedTarget) {
        ImGui::SetNextItemOpen(true);
        if (ImGui::TreeNode("ImGuizmo")) {
            static Eigen::Vector3d scale = Eigen::Vector3d::Ones();
            static Eigen::QuaternionD orientation = Eigen::QuaternionD::Identity();
            ImGuizmo::useWidget(selectedTarget->position->global, selectedTarget->orientation.has_value() ? selectedTarget->orientation->global : orientation,
                                scale, camera.getViewMatrix(), camera.getProjectionMatrix());
            ImGui::TreePop();
        }
    }

    ImGui::End();
}

void MAMPApp::mouseButtonCallback(double xPos, double yPos, int button, int action) {
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
                const auto firstLink =
                    robot.getFirstLinkHitByRay(robotState, camera.getRayFromScreenCoordinates(xPos, yPos), robot.showVisuals, robot.showSkeleton);
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