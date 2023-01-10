#include <lenny/samp/Plan.h>
#include <lenny/tools/Gui.h>
#include <lenny/tools/Renderer.h>
#include <lenny/tools/Trajectory.h>

namespace lenny::samp {

inline Eigen::VectorXd getBaseWeights(const uint& stateSize) {
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(stateSize);
    weights.segment(0, 3) = 10.0 * Eigen::Vector3d::Ones();
    weights.segment(3, 3) = 20.0 * Eigen::Vector3d::Ones();
    return weights;
}

Plan::Plan(const rapt::Agent::SPtr agent, const uint& numSteps, const double& deltaT, tools::Plot<PlotType>::F_addPlot f_addPlot)
    : agent(agent), numSteps(numSteps), deltaT(deltaT), f_addPlot(f_addPlot) {
    //Initialize motion trajectory
    initializeMotionTrajectory();

    //Initialize plots
    initializePlots();

    //Add state regularizer as default
    stateRegularizers.emplace_back(samp::StateRegularizer(agent, Eigen::VectorXd::Ones(agent->robot.getStateSize()), {0.0, 1.0}));
    stateRegularizers.emplace_back(samp::StateRegularizer(agent, getBaseWeights(agent->robot.getStateSize()), {0.0, 1.0}));
}

Plan::Plan(Plan&& other) : Plan(other.agent, other.numSteps, other.deltaT, other.f_addPlot) {}

void Plan::initializeMotionTrajectory() {
    const int stateSize = agent->getStateSize();
    const Eigen::VectorXd initialAgentState = agent->getInitialAgentState();
    motionTrajectory.resize(numSteps * stateSize);
    motionTrajectory.setZero();
    for (uint i = 0; i < numSteps; i++)
        motionTrajectory.segment(i * stateSize, stateSize) = initialAgentState;
}

Eigen::VectorXd Plan::getAgentStateForTrajectoryIndex(const int& index) const {
    return getAgentStateForTrajectoryIndex(motionTrajectory, index);
}

Eigen::VectorXd Plan::getAgentStateForTrajectoryTime(const double& time) const {
    return getAgentStateForTrajectoryTime(motionTrajectory, time);
}

int Plan::getTrajectoryIndexForTime(const double& time) const {
    int index = (int)std::round((time / getTotalTrajectoryTime()) * (double)numSteps) - 1;
    tools::utils::boundToRange(index, -1, (int)numSteps - 1);
    return index;
}

double Plan::getTrajectoryTimeForIndex(const int& index) const {
    return (double)index * deltaT;
}

double Plan::getTotalTrajectoryTime() const {
    return (double)numSteps * deltaT;
}

Eigen::VectorXd Plan::getAgentStateForTrajectoryIndex(const Eigen::VectorXd& motionTrajectory, const int& index) const {
    return agent->getAgentStateForTrajectoryIndex(rapt::Agent::MotionTrajectory(motionTrajectory, numSteps, deltaT), index);
}

Eigen::VectorXd Plan::getAgentStateForTrajectoryTime(const Eigen::VectorXd& motionTrajectory, const double& time) const {
    return agent->getAgentStateForTrajectoryTime(rapt::Agent::MotionTrajectory(motionTrajectory, numSteps, deltaT), time);
}

Eigen::VectorXd Plan::getAgentStateForTrajectoryPercentage(const Eigen::VectorXd& motionTrajectory, const double& percentage) const {
    return getAgentStateForTrajectoryIndex(motionTrajectory, getTrajectoryIndexForPercentage(percentage));
}

uint Plan::getTrajectoryIndexForPercentage(const double& percentage) const {
    if (percentage < 0.0)
        return 0;
    else if (percentage > 1.0)
        return numSteps - 1;
    int index = (int)std::round(percentage * (double)numSteps) - 1;
    tools::utils::boundToRange(index, 0, (int)numSteps - 1);
    return (uint)index;
}

void Plan::setNumSteps(const uint& numSteps) {
    //Store current trajectory
    tools::TrajectoryXd trajectory;
    for (int i = -1; i < (int)this->numSteps; i++)
        trajectory.addEntry((double)(i + 1) / (double)this->numSteps, getAgentStateForTrajectoryIndex(i));

    //Set num steps
    this->numSteps = numSteps;
    if (this->numSteps == 0)
        this->numSteps = 1;

    //Reinitialize motion trajectory
    const int stateSize = agent->getStateSize();
    motionTrajectory.resize(this->numSteps * stateSize);
    motionTrajectory.setZero();
    for (uint i = 0; i < this->numSteps; i++)
        motionTrajectory.segment(i * stateSize, stateSize) = trajectory.getLinearInterpolation((double)(i + 1) / (double)this->numSteps);
}

void Plan::setDeltaT(const double& deltaT) {
    //Set deltaT
    const double prevDeltaT = this->deltaT;
    this->deltaT = deltaT;
    if (this->deltaT < 1e-5)
        this->deltaT = 1e-5;

    //Update boundary conditions
    agent->setInitialRobotVelocityFromRobotVelocity(agent->getInitialRobotVelocity() * prevDeltaT / this->deltaT);
}

void Plan::setDofMask(const Eigen::VectorXb& dofMask) {
    //Store entire robot trajectory
    std::vector<Eigen::VectorXd> robotTrajectory;
    for (uint i = 0; i < numSteps; i++)
        robotTrajectory.emplace_back(agent->getRobotStateFromAgentState(getAgentStateForTrajectoryIndex(i)));

    //Set new dof mask
    agent->setDofMask(dofMask);

    //Reinitialize motion trajectory
    const int stateSize = agent->getStateSize();
    motionTrajectory.resize(numSteps * stateSize);
    motionTrajectory.setZero();
    for (uint i = 0; i < numSteps; i++)
        motionTrajectory.segment(i * stateSize, stateSize) = agent->getAgentStateFromRobotState(robotTrajectory.at(i));

    //Reinitialize plots
    initializePlots();
}

const uint& Plan::getNumSteps() const {
    return numSteps;
}

const double& Plan::getDeltaT() const {
    return deltaT;
}

void Plan::initializePlots() {
    positionPlots.clear();
    velocityPlots.clear();
    if (f_addPlot) {
        for (int i = 0; i < agent->getStateSize(); i++) {
            const std::string description = agent->getDescriptionForDofIndex(i);

            //--- positionPlots
            f_addPlot(positionPlots, description, "time", "value", 1000);
            positionPlots.back()->addLineSpec(
                {"lower_limit", [](const std::array<double, 3>& d) { return (float)d.at(0); }, std::array<float, 3>{1.0, 0.0, 0.0}});
            positionPlots.back()->addLineSpec(
                {"upper_limit", [](const std::array<double, 3>& d) { return (float)d.at(1); }, std::array<float, 3>{1.0, 0.0, 0.0}});
            positionPlots.back()->addLineSpec(
                {"angle", [](const std::array<double, 3>& d) { return (float)d.at(2); }, std::array<float, 3>{0.30196, 0.73333, 0.90196}});

            //--- velocityPlots
            f_addPlot(velocityPlots, description, "time", "value", 1000);
            velocityPlots.back()->addLineSpec(
                {"lower_limit", [](const std::array<double, 3>& d) { return (float)d.at(0); }, std::array<float, 3>{0.75, 0.0, 0.0}});
            velocityPlots.back()->addLineSpec(
                {"upper_limit", [](const std::array<double, 3>& d) { return (float)d.at(1); }, std::array<float, 3>{0.75, 0.0, 0.0}});
            velocityPlots.back()->addLineSpec(
                {"velocity", [](const std::array<double, 3>& d) { return (float)d.at(2); }, std::array<float, 3>{0.30196, 0.73333, 0.90196}});
        }
    }
}

void Plan::updatePlots(const bool& isRecedingHorizon) {
    auto updatePlots = [&](tools::Plot<PlotType>::List& plots, const robot::Robot::LIMITS_TYPE& limitsType, const Eigen::VectorXd& values,
                           const float& time) -> void {
        for (int j = 0; j < plots.size(); j++) {
            const auto& limit = agent->getLimitsForDofIndex(j, limitsType);
            if (limit.has_value())
                plots.at(j)->addData(time, {limit->first, limit->second, values[j]});
            else
                plots.at(j)->addData(time, {values[j], values[j], values[j]});
        }
    };

    if (isRecedingHorizon) {
        static float time = 0.f;
        const Eigen::VectorXd q_0 = getAgentStateForTrajectoryIndex(motionTrajectory, 0);
        const Eigen::VectorXd q_m1 = getAgentStateForTrajectoryIndex(motionTrajectory, -1);
        updatePlots(positionPlots, robot::Robot::POSITION, q_0, time);
        updatePlots(velocityPlots, robot::Robot::VELOCITY, (q_0 - q_m1) / deltaT, time);
        time += (float)deltaT;
    } else {
        //Clear data first
        for (auto& plot : positionPlots)
            plot->clearData();
        for (auto& plot : velocityPlots)
            plot->clearData();

        //Add new data
        for (int i = -1; i < (int)numSteps; i++) {
            const Eigen::VectorXd q_i = getAgentStateForTrajectoryIndex(motionTrajectory, i);
            const Eigen::VectorXd q_im1 = getAgentStateForTrajectoryIndex(motionTrajectory, i - 1);
            const float time = (float)(i + i) * (float)deltaT;
            updatePlots(positionPlots, robot::Robot::POSITION, q_i, time);
            updatePlots(velocityPlots, robot::Robot::VELOCITY, (q_i - q_im1) / deltaT, time);
        }
    }
}

void Plan::drawScene(const double& currentAnimationTime, const bool& isRecedingHorizon) const {
    agent->drawScene(rapt::Agent::MotionTrajectory(motionTrajectory, numSteps, deltaT), currentAnimationTime, isRecedingHorizon);
    if (showEndEffectorTrajectories)
        drawEndEffectorTrajectories();
    if (showSkeletonTrajectory)
        drawSkeletonTrajectory();
    if (showVisualsTrajectory)
        drawVisualsTrajectory();
    if (showLinkTargets)
        drawLinkTargets();
    if (showStateTargets)
        drawStateTargets();
}

void Plan::drawEndEffectorTrajectories() const {
    for (const auto& [eeName, ee] : agent->robot.endEffectors) {
        std::vector<Eigen::Vector3d> points;
        for (int i = -1; i < (int)numSteps; i += (int)trajectoryDrawingInterval)
            points.emplace_back(agent->computeGlobalPoint(getAgentStateForTrajectoryIndex(i), ee.localGraspTrafo.position, ee.linkName));
        tools::Renderer::I->drawTrajectory(points, 0.01, Eigen::Vector4d(0.0, 0.0, 0.75, 0.5), true);
    }
}

void Plan::drawSkeletonTrajectory() const {
    Eigen::Vector4d linkColor, jointColor;
    linkColor << robot::Link::skeletonColor, 0.5;
    jointColor << robot::Joint::skeletonColor, 0.5;
    for (int i = -1; i < (int)numSteps; i += (int)trajectoryDrawingInterval) {
        const Eigen::VectorXd agentState = getAgentStateForTrajectoryIndex(i);
        const Eigen::VectorXd robotState = agent->getRobotStateFromAgentState(agentState);
        agent->robot.drawSkeleton(robotState, agent->robot.skeletonRadius, linkColor, jointColor);
    }
}

void Plan::drawVisualsTrajectory() const {
    for (int i = -1; i < (int)numSteps; i += (int)trajectoryDrawingInterval) {
        const Eigen::VectorXd agentState = getAgentStateForTrajectoryIndex(i);
        agent->drawVisuals(agentState, std::nullopt, 0.5);
    }
}

void Plan::drawLinkTargets() const {
    using tools::Renderer;
    for (const LinkTarget& target : linkTargets) {
        const Eigen::VectorXd agentState = getAgentStateForTrajectoryPercentage(motionTrajectory, target.step.get());

        if (target.position.has_value()) {
            Renderer::I->drawSphere(target.position->global, 0.01, Eigen::Vector4d(0.75, 0.75, 0.0, 0.75));
            const Eigen::Vector3d globalPosition = agent->computeGlobalPoint(agentState, target.position->local, target.linkName);
            Renderer::I->drawArrow(globalPosition, (target.position->global - globalPosition), 0.005, Eigen::Vector4d(0.75, 0.75, 0.0, 0.75));
        }

        if (target.orientation.has_value()) {
            Eigen::Vector3d origin =
                target.position.has_value() ? target.position->global : agent->computeGlobalPoint(agentState, Eigen::Vector3d::Zero(), target.linkName);
            Renderer::I->drawCoordinateSystem(origin, target.orientation->global, 0.025, 0.001);
            const Eigen::QuaternionD globalOrientation = agent->computeGlobalOrientation(agentState, target.orientation->local, target.linkName);
            Renderer::I->drawCoordinateSystem(origin, globalOrientation, 0.025, 0.001);
        }
    }
}

void Plan::drawStateTargets() const {
    for (const StateTarget& target : stateTargets) {
        const Eigen::VectorXd agentState = getAgentStateForTrajectoryPercentage(motionTrajectory, target.step.get());
        const Eigen::VectorXd robotState = agent->getRobotStateFromAgentState(agentState);
        agent->robot.drawSkeleton(target.getState(), agent->robot.skeletonRadius, Eigen::Vector4d(0.75, 0.75, 0.0, 0.75),
                                  Eigen::Vector4d(0.75, 0.75, 0.0, 0.75));
        agent->robot.drawSkeleton(robotState, agent->robot.skeletonRadius, Eigen::Vector4d(0.75, 0.75, 0.0, 0.75), Eigen::Vector4d(0.75, 0.75, 0.0, 0.75));
    }
}

void Plan::drawGui(const bool& withTrajectoryStettings) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Plan - `" + agent->name + "`").c_str())) {
        if (Gui::I->TreeNode("Settings")) {
            if (withTrajectoryStettings) {
                int numSteps = this->numSteps;
                if (Gui::I->Slider("Num Steps", numSteps, 1, 100))
                    setNumSteps(numSteps);

                double deltaT = this->deltaT;
                if (Gui::I->Slider("Delta T", deltaT, 1e-5, 0.1))
                    setDeltaT(deltaT);
            }

            Eigen::VectorXb dofMask = agent->getDofMask();
            if (Gui::I->TreeNode("Dof Mask")) {
                for (int i = 0; i < dofMask.size(); i++)
                    if (Gui::I->Checkbox(agent->robot.getDescriptionForDofIndex(i).c_str(), dofMask[i]))
                        setDofMask(dofMask);

                Gui::I->TreePop();
            }

            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Agenda")) {
            Gui::I->PushItemWidth(75.f);

            if (Gui::I->TreeNode("Link Targets")) {
                int iter = 0;
                for (auto& entry : linkTargets)
                    entry.drawGui(std::to_string(iter++));
                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Link Regularizers")) {
                int iter = 0;
                for (auto& entry : linkRegularizers)
                    entry.drawGui(std::to_string(iter++));
                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("State Targets")) {
                int iter = 0;
                for (auto& entry : stateTargets)
                    entry.drawGui(std::to_string(iter++));
                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("State Regularizers")) {
                int iter = 0;
                for (auto& entry : stateRegularizers)
                    entry.drawGui(std::to_string(iter++));
                Gui::I->TreePop();
            }

            Gui::I->PopItemWidth();
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Drawing")) {
            Gui::I->Checkbox("Show EndEffector Trajectories", showEndEffectorTrajectories);
            Gui::I->Checkbox("Show Skeleton Trajectory", showSkeletonTrajectory);
            Gui::I->Checkbox("Show Visuals Trajectory", showVisualsTrajectory);
            Gui::I->Checkbox("Show Link Targets", showLinkTargets);
            Gui::I->Checkbox("Show State Targets", showStateTargets);

            Gui::I->Slider("Trajectory Drawing Interval", trajectoryDrawingInterval, 1, numSteps - 1);

            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Plots")) {
            if (Gui::I->TreeNode("Positions")) {
                for (auto& plot : positionPlots)
                    plot->draw();
                Gui::I->TreePop();
            }

            if (Gui::I->TreeNode("Velocities")) {
                for (auto& plot : velocityPlots)
                    plot->draw();
                Gui::I->TreePop();
            }

            Gui::I->TreePop();
        }

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp