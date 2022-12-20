#pragma once

#include <lenny/optimization/TotalObjective.h>
#include <lenny/rapt/WorldCollisionHandler.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

class TotalObjective : public optimization::TotalObjective {
public:
    TotalObjective(const Plan& plan, const rapt::WorldCollisionHandler::PrimitiveList& worldCollisionPrimitives,
                   const std::string& description = "Total SAMP Objective", const double& regularizerWeight = 1e-5);
    TotalObjective(TotalObjective&&) = default;
    ~TotalObjective() = default;

    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

public:
    const Plan& plan;
};

}  // namespace lenny::samp
