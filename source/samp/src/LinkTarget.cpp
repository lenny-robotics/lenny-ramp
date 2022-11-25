#include <lenny/samp/LinkTarget.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {
LinkTarget::LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step, const Position& position,
                       const Orientation& orientation)
    : linkName(linkName), step(step), position(position), orientation(orientation) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

LinkTarget::LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step, const Position& position)
    : linkName(linkName), step(step), position(position), orientation(std::nullopt) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

LinkTarget::LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step, const Orientation& orientation)
    : linkName(linkName), step(step), position(std::nullopt), orientation(orientation) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

LinkTarget::LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step)
    : linkName(linkName), step(step), position(std::nullopt), orientation(std::nullopt) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

void LinkTarget::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Link Target - " + description).c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());
        step.drawGui("Step");

        if (Gui::I->TreeNode("Position")) {
            if (position.has_value()) {
                Gui::I->Input("Global", position->global);
                Gui::I->Input("Local", position->local);
                Gui::I->Input("Weights", position->weights);
            } else {
                if (Gui::I->Button("Deactivate"))
                    position = std::nullopt;
            }
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Orientation")) {
            if (orientation.has_value()) {
                Gui::I->Slider("Global", orientation->global);
                Gui::I->Slider("Local", orientation->local);
                Gui::I->Input("Weights", orientation->weights);
            } else {
                if (Gui::I->Button("Deactivate"))
                    orientation = std::nullopt;
            }
            Gui::I->TreePop();
        }

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp