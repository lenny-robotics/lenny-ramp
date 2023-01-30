#include <lenny/samp/LinkLimits.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

template <typename T>
void LinkLimits::Limits<T>::Limits::drawGui() {
    using tools::Gui;
    Gui::I->Input("Local", local);
    Gui::I->Input("Lower", lower);
    Gui::I->Input("Upper", upper);
}

LinkLimits::LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range, const Linear& linear, const Angular& angular)
    : linkName(linkName), range(range), linear(linear), angular(angular) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

LinkLimits::LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range, const Linear& linear)
    : linkName(linkName), range(range), linear(linear), angular(std::nullopt) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

LinkLimits::LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range, const Angular& angular)
    : linkName(linkName), range(range), linear(std::nullopt), angular(angular) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

LinkLimits::LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range)
    : linkName(linkName), range(range), linear(std::nullopt), angular(std::nullopt) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

void LinkLimits::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Link Limits - " + description).c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());
        range.drawGui("Range");

        if (Gui::I->TreeNode("Linear")) {
            if (linear.has_value()) {
                linear->drawGui();
                if (Gui::I->Button("Deactivate"))
                    linear = std::nullopt;
            }
            Gui::I->TreePop();
        }

        if (Gui::I->TreeNode("Angular")) {
            if (angular.has_value()) {
                angular->drawGui();
                if (Gui::I->Button("Deactivate"))
                    angular = std::nullopt;
            }
            Gui::I->TreePop();
        }

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp