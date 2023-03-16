#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/PercentageRange.h>
#include <lenny/tools/Definitions.h>

namespace lenny::samp {

class LinkLimits {
public:
    //--- Limits
    template <typename T>
    struct Limits {
        Limits(const T& local, const Eigen::Vector3d& lower, const Eigen::Vector3d& upper) : local(local), lower(lower), upper(upper) {}

        void drawGui();

        T local;
        Eigen::Vector3d lower, upper;
    };
    typedef Limits<Eigen::Vector3d> Linear;
    typedef Limits<Eigen::QuaternionD> Angular;

    //--- Constructor
    LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range, const Linear& linear, const Angular& angular);
    LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range, const Linear& linear);
    LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range, const Angular& angular);
    LinkLimits(const rapt::Agent::CSPtr& agent, const std::string& linkName, const PercentageRange& range);
    ~LinkLimits() = default;

    //--- Gui
    void drawGui(const std::string& description);

public:
    //--- Members
    const std::string linkName;      //Name of the target link
    PercentageRange range;           //Trajectory range when limits are applied
    std::optional<Linear> linear;    //Linear limits
    std::optional<Angular> angular;  //Angular limits
};

}  // namespace lenny::samp