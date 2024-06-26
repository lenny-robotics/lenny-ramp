#include "CollisionApp.h"

#include <lenny/gui/Gui.h>
#include <lenny/gui/Guizmo.h>
#include <lenny/gui/ImGui.h>

namespace lenny {

CollisionApp::CollisionApp() : gui::Application("CollisionApp") {
    //Setup process
    processes.emplace_back(std::make_shared<gui::Process>(
        "Process-1", [&]() -> void { process(); }, [&]() -> void { restart(); }));

    //Setup scene
    const auto [width, height] = getCurrentWindowSize();
    scenes.emplace_back(std::make_shared<gui::Scene>("Scene-1", width, height));
    scenes.back()->f_drawScene = [&]() -> void { drawScene(); };
    scenes.back()->showOrigin = false;
    scenes.back()->showGround = false;

    //Setup drawing
    agent->showCollisionPrimitives = true;

    //Set target
    const samp::LinkTarget::Position positionTarget(Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::Ones());
    const samp::LinkTarget::Orientation orientationTarget(Eigen::QuaternionD::Identity(), Eigen::QuaternionD::Identity(), Eigen::Vector3d::Ones());
    planner.plan.linkTargets.emplace_back(agent, "base", 1.0, positionTarget, orientationTarget);

    //Initialize collision primitives
    setupAgentPrimitive();
    setupWorldPrimitive();
}

void CollisionApp::restart() {
    planner.plan.initializeMotionTrajectory();
}

void CollisionApp::process() {
    planner.solve(1);
}

void CollisionApp::drawScene() const {
    planner.drawScene();
    worldCollisionHandler.drawScene();
}

void CollisionApp::drawGui() {
    gui::Application::drawGui();

    ImGui::Begin("Main Menu");

    robot.drawGui(false);
    agent->drawGui(true);
    planner.drawGui();
    worldCollisionHandler.drawGui();

    if (gui::Gui::I->EnumSelection<PRIMITIVE>("Agent Base Primitive", agentBasePrimitive))
        setupAgentPrimitive();

    if (gui::Gui::I->EnumSelection<PRIMITIVE>("Agent Link Primitive", agentLinkPrimitive))
        setupAgentPrimitive();

    if (gui::Gui::I->EnumSelection<PRIMITIVE>("World Primitive", worldPrimitive))
        setupWorldPrimitive();

    ImGui::End();
}

void CollisionApp::drawGuizmo() {
    if (worldCollisionHandler.primitives.size() > 0) {
        auto& [primitive, parentState] = worldCollisionHandler.primitives.back();
        static Eigen::Vector3d scale = Eigen::Vector3d::Ones();
        tools::Transformation trafo = worldCollisionHandler.convertPose(parentState);
        gui::Guizmo::useWidget(trafo.position, trafo.orientation, scale);
        parentState = worldCollisionHandler.convertPose(trafo);
    }
}

void CollisionApp::setupAgentPrimitive() {
    agent->collisionPrimitives.clear();

    auto setAgentPrimitive = [&](const PRIMITIVE& primitive, const std::string& linkName, const double& dimension) -> void {
        switch (primitive) {
            case SPHERE: {
                agent->addCollisionSphere(linkName, Eigen::Vector3d::Zero(), 0.5 * dimension);
                break;
            }
            case CAPSULE: {
                agent->addCollisionCapsule(linkName, Eigen::Vector3d(-0.5 * dimension, 0.0, 0.0), Eigen::Vector3d(0.5 * dimension, 0.0, 0.0), 0.5 * dimension);
                break;
            }
            case RECTANGLE: {
                agent->addCollisionRectangle(linkName, Eigen::Vector3d::Zero(), Eigen::QuaternionD::Identity(), dimension * Eigen::Vector2d::Ones(),
                                             0.1 * dimension);
                break;
            }
            case BOX: {
                agent->addCollisionBox(linkName, Eigen::Vector3d::Zero(), Eigen::QuaternionD::Identity(), dimension * Eigen::Vector3d::Ones(), 0.1 * dimension);
                break;
            }
        }
    };

    setAgentPrimitive(agentBasePrimitive, "base", 0.2);
    setAgentPrimitive(agentLinkPrimitive, "link2", 0.1);
}

void CollisionApp::setupWorldPrimitive() {
    worldCollisionHandler.primitives.clear();

    switch (worldPrimitive) {
        case SPHERE: {
            worldCollisionHandler.addCollisionSphere(Eigen::Vector6d::Zero(), 0.1);
            break;
        }
        case CAPSULE: {
            worldCollisionHandler.addCollisionCapsule(Eigen::Vector6d::Zero(), 0.2, 0.1);
            break;
        }
        case RECTANGLE: {
            worldCollisionHandler.addCollisionRectangle(Eigen::Vector6d::Zero(), 0.2 * Eigen::Vector2d::Ones(), 0.01);
            break;
        }
        case BOX: {
            worldCollisionHandler.addCollisionBox(Eigen::Vector6d::Zero(), 0.2 * Eigen::Vector3d::Ones(), 0.01);
            break;
        }
    }
}

}  // namespace lenny