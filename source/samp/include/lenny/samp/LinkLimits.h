#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/PercentageRange.h>
#include <lenny/tools/Definitions.h>

namespace lenny::samp {

class LinkLimits {
public:
    //--- Limits
    struct Limits {
        struct Entry {
            Entry(const Eigen::Vector3d& lower, const Eigen::Vector3d& upper, const Eigen::Vector3d& weights) : lower(lower), upper(upper), weights(weights) {}
            Eigen::Vector3d lower, upper, weights;

            void drawGui(const std::string& description);
        };

        Limits(const PercentageRange& range, const Entry& linear, const Entry& angular) : range(range), linear(linear), angular(angular) {}

        void drawGui(const std::string& description);

        PercentageRange range;
        std::optional<Entry> linear;
        std::optional<Entry> angular;
    };

    //--- Constructor
    LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName);
    ~LinkLimits() = default;

    //--- Gui
    void drawGui(const std::string& description);

public:
    //--- Members
    const std::string linkName;                              //Name of the target link
    std::optional<Limits> position, velocity, acceleration;  //Limits
};

}  // namespace lenny::samp