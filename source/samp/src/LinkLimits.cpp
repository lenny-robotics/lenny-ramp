#include <lenny/samp/LinkLimits.h>
#include <lenny/tools/Gui.h>

namespace lenny::samp {

template <typename T_LIMITS>
void LinkLimits::Limits<T_LIMITS>::Entry::drawGui(const std::string& description) {
    using tools::Gui;
    if (Gui::I->TreeNode(description.c_str())) {
        Gui::I->Input("Lower", lower);
        Gui::I->Input("Upper", upper);
        Gui::I->Input("Weights", weights);

        Gui::I->TreePop();
    }
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
    if (Gui::I->TreeNode(("Link Target - " + description).c_str())) {
        Gui::I->Text("Link: %s", linkName.c_str());
        range.drawGui("Range");

        if (linear.has_value()) {
            if (Gui::I->TreeNode("Linear")) {
                Gui::I->Input("Local", linear->local);
                if (linear->position.has_value())
                    linear->position->drawGui("Position");
                if (linear->velocity.has_value())
                    linear->velocity->drawGui("Velocity");
                if (linear->acceleration.has_value())
                    linear->acceleration->drawGui("Acceleration");
                if (Gui::I->Button("Deactivate"))
                    linear = std::nullopt;
                Gui::I->TreePop();
            }
        }

        if (angular.has_value()) {
            if (Gui::I->TreeNode("Angular")) {
                Gui::I->Slider("Local", angular->local);
                if (angular->position.has_value())
                    angular->position->drawGui("Position");
                if (angular->velocity.has_value())
                    angular->velocity->drawGui("Velocity");
                if (angular->acceleration.has_value())
                    angular->acceleration->drawGui("Acceleration");
                if (Gui::I->Button("Deactivate"))
                    angular = std::nullopt;
                Gui::I->TreePop();
            }
        }

        Gui::I->TreePop();
    }
}

}  // namespace lenny::samp