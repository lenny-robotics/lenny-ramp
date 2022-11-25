#include <lenny/samp/StateTarget.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

StateTarget::StateTarget(const rapt::Agent::CSPtr agent, const Eigen::VectorXd& state, const Eigen::VectorXd& weights, const double& step)
    : agent(agent), step(step) {
    setState(state);
    setWeights(weights);
}

void StateTarget::setState(const Eigen::VectorXd& state) {
    if (state.size() != agent->robot.getStateSize())
        LENNY_LOG_ERROR("Wrong input size: %d VS %d", state.size(), agent->robot.getStateSize());
    this->state = state;
}

void StateTarget::setWeights(const Eigen::VectorXd& weights) {
    if (weights.size() != agent->robot.getStateSize())
        LENNY_LOG_ERROR("Wrong input size: %d VS %d", weights.size(), agent->robot.getStateSize());
    this->weights = weights;
}

const Eigen::VectorXd& StateTarget::getState() const {
    return state;
}

const Eigen::VectorXd& StateTarget::getWeights() const {
    return weights;
}

void StateTarget::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("State Target - " + description).c_str())) {
        Gui::I->Input("State", state);
        Gui::I->Input("Weights", weights);
        step.drawGui("Step");

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp