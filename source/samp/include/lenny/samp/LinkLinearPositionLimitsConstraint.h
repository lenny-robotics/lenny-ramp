#pragma once

#include <lenny/samp/LinkLimitsConstraint.h>

namespace lenny::samp {

class LinkLinearPositionLimitsConstraint : public LinkLimitsConstraint<Eigen::Vector3d> {
public:
    LinkLinearPositionLimitsConstraint(const Plan& plan);
    ~LinkLinearPositionLimitsConstraint() = default;

    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

private:
    Eigen::Vector3d computeValue(const Eigen::VectorXd& q, const std::string& linkName, const Eigen::Vector3d& local, const uint& index) const override;
    void computeJacobian(Eigen::TripletDList& jacobian, const Eigen::VectorXd& q, const std::string& linkName, const Eigen::Vector3d& local,
                         const uint& index) const override;
    void computeTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& q, const std::string& linkName, const Eigen::Vector3d& local,
                       const uint& index) const override;
};

}  // namespace lenny::samp