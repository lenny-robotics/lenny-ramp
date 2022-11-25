#include <lenny/samp/LinkRegularizer.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

LinkRegularizer::LinkRegularizer(const rapt::Agent::CSPtr agent, const std::string& linkName, const Eigen::Vector3d& localCoordinates, const Eigen::Vector3d& weights,
                                 const std::pair<double, double>& range)
    : linkName(linkName), localCoordinates(localCoordinates), weights(weights), range(range) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

void LinkRegularizer::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Link Regularizer - " + description).c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());
        Gui::I->Input("Local Coordinates", localCoordinates);
        Gui::I->Input("Weights", weights);
        range.drawGui("Range");
        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp