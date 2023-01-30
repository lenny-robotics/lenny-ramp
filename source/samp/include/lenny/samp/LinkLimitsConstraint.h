#pragma once

#include <lenny/optimization/InequalityConstraint.h>
#include <lenny/samp/Plan.h>

namespace lenny::samp {

template <typename T>
class LinkLimitsConstraint : public optimization::InequalityConstraint {
public:
    LinkLimitsConstraint(const std::string& description, const Plan& plan, const double& stiffness, const double& testFactor);
    virtual ~LinkLimitsConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

protected:
    void setupLimitInfos(const Eigen::VectorXd& q, const std::vector<LinkLimits>& linkLimitsList, const double& dt) const;

    virtual Eigen::Vector3d computeValue(const Eigen::VectorXd& q, const std::string& linkName, const T& local, const uint& index) const = 0;
    virtual void computeJacobian(Eigen::TripletDList& jacobian, const Eigen::VectorXd& q, const std::string& linkName, const T& local,
                                 const uint& index) const = 0;
    virtual void computeTensor(Eigen::TensorD& tensor, const Eigen::VectorXd& q, const std::string& linkName, const T& local, const uint& index) const = 0;

public:
    double testFactor;  //Set by constructor
    bool printLimitInfos = false;

protected:
    const Plan& plan;

    enum BOUND_TYPE { LOWER, UPPER };
    typedef std::tuple<LinkLimits::Limits<T>, std::string, BOUND_TYPE, uint> LimitInfo;  //[limits, linkName, bound_type, index]
    mutable std::vector<LimitInfo> limitInfos;
};

}  // namespace lenny::samp