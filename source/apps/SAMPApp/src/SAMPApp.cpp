#include "SAMPApp.h"

#include <lenny/gui/ImGui.h>
#include <lenny/gui/ImGuizmo.h>

namespace lenny {

SAMPApp::SAMPApp() : gui::Application("SAMPApp") {
    agent->showCollisionPrimitives = true;
    planner.animator.setCurrentTimeFromPercentage(1.0);
}

void SAMPApp::restart() {
    planner.plan.initializeMotionTrajectory();
}

void SAMPApp::process() {
    if (planner.isRecedingHorizon)
        planner.plan.setDeltaT(1.0 / targetFramerate);
    planner.solve(1);
}

void SAMPApp::drawScene() const {
    planner.drawScene();
    worldCollisionHandler.drawScene();
}

void SAMPApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    if (ImGui::TreeNode("App Settings")) {
        ImGui::Checkbox("Use orientation targets", &useOrientationTargets);
        ImGui::Checkbox("Use multiple targets", &useMultipleTargets);
        ImGui::TreePop();
    }

    robot.drawGui(false);
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

void SAMPApp::mouseButtonCallback(double xPos, double yPos, int button, int action) {
    if (action == GLFW_PRESS) {
        selectedTarget = nullptr;
        std::vector<samp::LinkTarget>& linkTargets = planner.plan.linkTargets;

        //Right mouse button
        if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            if (linkTargets.size() > 0)
                linkTargets.pop_back();
        }

        //Left mouse button
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (!useMultipleTargets)
                linkTargets.clear();

            const Eigen::VectorXd robotState = agent->getRobotStateFromAgentState(planner.plan.getAgentStateForTrajectoryTime(planner.animator.currentTime));
            const auto firstLink =
                robot.getFirstLinkHitByRay(robotState, camera.getRayFromScreenCoordinates(xPos, yPos), robot.showVisuals, robot.showSkeleton);
            if (firstLink.has_value()) {
                const auto& [linkName, selectedPoint] = firstLink.value();

                const samp::LinkTarget::Position positionTarget(selectedPoint, robot.computeLocalPoint(robotState, selectedPoint, linkName),
                                                                Eigen::Vector3d::Ones());
                if (useOrientationTargets) {
                    const samp::LinkTarget::Orientation orientationTarget(robot.computeGlobalOrientation(robotState, Eigen::QuaternionD::Identity(), linkName),
                                                                          Eigen::QuaternionD::Identity(), Eigen::Vector3d::Ones());

                    linkTargets.emplace_back(agent, linkName, 1.0, positionTarget, orientationTarget);
                } else {
                    linkTargets.emplace_back(agent, linkName, 1.0, positionTarget);
                }

                selectedTarget = &linkTargets.back();
            }
        }
    }
}

}  // namespace lenny