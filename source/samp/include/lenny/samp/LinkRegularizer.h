#pragma once

#include <lenny/rapt/Agent.h>
#include <lenny/samp/PercentageRange.h>
#include <lenny/tools/Definitions.h>

namespace lenny::samp {

class LinkRegularizer {
public:
    LinkRegularizer(const rapt::Agent::CSPtr agent, const std::string& linkName, const Eigen::Vector3d& localCoordinates, const Eigen::Vector3d& weights,
                    const std::pair<double, double>& range);
    ~LinkRegularizer() = default;

    void drawGui(const std::string& description);

public:
    const std::string linkName;        //Name of the target link
    Eigen::Vector3d localCoordinates;  //Local coordinates expressed in the coordinate frame of the link
    Eigen::Vector3d weights;           //Weights for individual directions
    PercentageRange range;             //Trajectory range when regularization is applied
};

}  // namespace lenny::samp