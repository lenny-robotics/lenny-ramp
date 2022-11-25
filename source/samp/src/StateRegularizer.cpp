#include <lenny/samp/StateRegularizer.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

StateRegularizer::StateRegularizer(const rapt::Agent::CSPtr agent, const Eigen::VectorXd& weights, const std::pair<double, double>& range)
    : agent(agent), range(range) {
    setWeights(weights);
}

void StateRegularizer::setWeights(const Eigen::VectorXd& weights) {
    if (weights.size() != agent->robot.getStateSize())
        LENNY_LOG_ERROR("Wrong input size: %d VS %d", weights.size(), agent->robot.getStateSize());
    this->weights = weights;
}

const Eigen::VectorXd& StateRegularizer::getWeights() const {
    return weights;
}

void StateRegularizer::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("State Regularizer - " + description).c_str())) {
        Gui::I->Input("Weights", weights);
        range.drawGui("Range");

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp