#pragma once

#include <lenny/optimization/InequalityConstraint.h>
#include <lenny/samp/Plan.h>

#include <unordered_map>

namespace lenny::mamp {

class AgentCollisionAvoidanceConstraint : public optimization::InequalityConstraint {
public:
    AgentCollisionAvoidanceConstraint(const std::vector<samp::Plan>& plans);
    ~AgentCollisionAvoidanceConstraint() = default;

    uint getConstraintNumber() const override;
    void computeConstraint(Eigen::VectorXd& C, const Eigen::VectorXd& q) const override;
    void computeJacobian(Eigen::SparseMatrixD& pCpQ, const Eigen::VectorXd& q) const override;
    void computeTensor(Eigen::TensorD& p2CpQ2, const Eigen::VectorXd& q) const override;

    void preFDEvaluation(const Eigen::VectorXd& q) const override;
    bool preValueEvaluation(const Eigen::VectorXd& q) const override;
    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

    void drawGuiContent() override;

private:
    std::pair<uint, uint> getInfosForPlanIndex(const uint& planIndex) const;                                      //[startIndex, size]
    uint getTrajectoryStepForPlanIndex(const uint& planIndex, const std::pair<uint, uint>& otherPlanInfo) const;  //[plan list index, trajectory step]
    void setupPairList(const Eigen::VectorXd& q) const;                                                           //Also directly updates the corresponding t's
    void updateTs(const Eigen::VectorXd& q, const bool& forFD = false) const;

public:
    double neighborRadius = 0.1;
    bool useParentTensor = false;
    bool printPrimitivePairs = false;

private:
    const std::vector<samp::Plan>& plans;

    typedef std::tuple<uint, uint, collision::Primitive::SPtr> Primitive;                 //[plan list index, trajectory step, collision primitive]
    typedef std::tuple<const Primitive, const Primitive, Eigen::VectorXd> PrimitivePair;  //[primitive A, primitive B, t]
    mutable std::vector<PrimitivePair> pairList;
};

}  // namespace lenny::mamp