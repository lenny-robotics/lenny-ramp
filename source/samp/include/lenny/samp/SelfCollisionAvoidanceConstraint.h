#pragma once

#include <lenny/optimization/InequalityConstraint.h>
#include <lenny/samp/Plan.h>

#include <unordered_map>

namespace lenny::samp {

class SelfCollisionAvoidanceConstraint : public optimization::InequalityConstraint {
public:
    SelfCollisionAvoidanceConstraint(const Plan& plan);
    ~SelfCollisionAvoidanceConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

    bool preValueEvaluation(const Eigen::VectorXd& q) const override;
    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

private:
    void setupPairList(const Eigen::VectorXd& q) const;  //Also directly updates the corresponding t's
    void updateTs(const Eigen::VectorXd& q, const bool& forFD = false) const;

public:
    double neighborRadius = 0.1;
    bool useParentTensor = false;
    bool printPrimitivePairs = false;

private:
    const Plan& plan;

    typedef std::tuple<const collision::Primitive::SPtr, const collision::Primitive::SPtr, Eigen::VectorXd> PrimitivePair;  //[Agent A, Agent B, t]
    mutable std::unordered_map<uint, std::vector<PrimitivePair>> pairList;                                                  //[trajectoryStep, PrimitivePair]
};

}  // namespace lenny::samp