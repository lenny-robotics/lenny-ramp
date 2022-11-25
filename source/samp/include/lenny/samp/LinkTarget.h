#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/Percentage.h>
#include <lenny/tools/Definitions.h>

namespace lenny::samp {

class LinkTarget {
public:
    //--- Target
    template <typename T>
    struct Target {
        Target(const T& global, const T& local, const Eigen::Vector3d& weights) : global(global), local(local), weights(weights) {}

        T global;
        T local;
        Eigen::Vector3d weights;
    };
    typedef Target<Eigen::Vector3d> Position;
    typedef Target<Eigen::QuaternionD> Orientation;

    //--- Constructor
    LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step, const Position& position, const Orientation& orientation);
    LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step, const Position& position);
    LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step, const Orientation& orientation);
    LinkTarget(const rapt::Agent::CSPtr& agent, const std::string& linkName, const double& step);
    ~LinkTarget() = default;

    //--- Gui
    void drawGui(const std::string& description);

public:
    //--- Members
    const std::string linkName;              //Name of the target link
    Percentage step;                         //Trajectory step when target is applied
    std::optional<Position> position;        //Position target
    std::optional<Orientation> orientation;  //Orientation target
};

}  // namespace lenny::samp