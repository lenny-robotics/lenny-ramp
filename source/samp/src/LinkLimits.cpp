#include <lenny/samp/LinkLimits.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

void LinkLimits::Limits::Entry::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(description.c_str())) {
        Gui::I->Input("Lower", lower);
        Gui::I->Input("Upper", upper);
        Gui::I->Input("Weights", weights);

        Gui::I->TreePop();
    }
}

void LinkLimits::Limits::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(description.c_str())) {
        range.drawGui("Range");
        if (linear.has_value())
            linear->drawGui("Linear");
        if (angular.has_value())
            linear->drawGui("Angular");
        Gui::I->TreePop();
    }
}

LinkLimits::LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName)
    : linkName(linkName), position(std::nullopt), velocity(std::nullopt), acceleration(std::nullopt) {
    if (agent->robot.links.find(linkName) == agent->robot.links.end())
        LENNY_LOG_ERROR("Link with name `%s` does not seem to belong to the agent with name `%s`", linkName.c_str(), agent->name.c_str());
}

void LinkLimits::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(("Link Target - " + description).c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());

        if (position.has_value())
            position->drawGui("Position");
        if (velocity.has_value())
            velocity->drawGui("Velocity");
        if (acceleration.has_value())
            velocity->drawGui("Acceleration");

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp