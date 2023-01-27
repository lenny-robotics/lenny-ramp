#include <lenny/samp/LinkAccelerationRegularizerObjective.h>
#include <lenny/samp/LinkOrientationTargetConstraint.h>
#include <lenny/samp/LinkPositionTargetConstraint.h>
#include <lenny/samp/LinkVelocityRegularizerObjective.h>
#include <lenny/samp/SelfCollisionAvoidanceConstraint.h>
#include <lenny/samp/StateAccelerationLimitsConstraint.h>
#include <lenny/samp/StateAccelerationRegularizerObjective.h>
#include <lenny/samp/StatePositionLimitsConstraint.h>
#include <lenny/samp/StateTargetConstraint.h>
#include <lenny/samp/StateVelocityLimitsConstraint.h>
#include <lenny/samp/StateVelocityRegularizerObjective.h>
#include <lenny/samp/TotalObjective.h>
#include <lenny/samp/WorldCollisionAvoidanceConstraint.h>

namespace lenny::samp {

TotalObjective::TotalObjective(const Plan& plan, const rapt::WorldCollisionHandler::PrimitiveList& worldCollisionPrimitives, const std::string& description,
                               const double& regularizerWeight)
    : optimization::TotalObjective(description, regularizerWeight), plan(plan) {
    subObjectives.clear();
    subObjectives.emplace_back(std::make_pair(std::make_unique<LinkPositionTargetConstraint>(plan), 1.0));
    subObjectives.emplace_back(std::make_pair(std::make_unique<LinkOrientationTargetConstraint>(plan), 0.1));
    subObjectives.emplace_back(std::make_pair(std::make_unique<StateTargetConstraint>(plan), 1.0));
    subObjectives.emplace_back(std::make_pair(std::make_unique<StateVelocityRegularizerObjective>(plan), 0.01));
    subObjectives.emplace_back(std::make_pair(std::make_unique<StateAccelerationRegularizerObjective>(plan), 0.01));
    subObjectives.emplace_back(std::make_pair(std::make_unique<LinkVelocityRegularizerObjective>(plan), 0.01));
    subObjectives.emplace_back(std::make_pair(std::make_unique<LinkAccelerationRegularizerObjective>(plan), 0.01));
    subObjectives.emplace_back(std::make_pair(std::make_unique<StatePositionLimitsConstraint>(plan), 10.0));
    subObjectives.emplace_back(std::make_pair(std::make_unique<StateVelocityLimitsConstraint>(plan), 10.0));
    subObjectives.emplace_back(std::make_pair(std::make_unique<StateAccelerationLimitsConstraint>(plan), 10.0));
    subObjectives.emplace_back(std::make_pair(std::make_unique<SelfCollisionAvoidanceConstraint>(plan), 10.0));
    subObjectives.emplace_back(std::make_pair(std::make_unique<WorldCollisionAvoidanceConstraint>(plan, worldCollisionPrimitives), 10.0));
}

void TotalObjective::preDerivativeEvaluation(const Eigen::VectorXd& q) const {
    optimization::TotalObjective::preDerivativeEvaluation(q);

    for (const auto& [objective, weight] : subObjectives)
        if (const optimization::Constraint* con = dynamic_cast<optimization::Constraint*>(objective.get()))
            con->softificationWeights.setOnes(con->getConstraintNumber());
}

}  // namespace lenny::samp