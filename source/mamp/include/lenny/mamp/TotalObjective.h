#pragma once

#include <lenny/mamp/MotionTrajectoryHandler.h>
#include <lenny/samp/TotalObjective.h>

namespace lenny::mamp {

class TotalObjective : public optimization::TotalObjective {
public:
    //--- Constructor
    TotalObjective(const MotionTrajectoryHandler& trajectoryHandler);
    ~TotalObjective() = default;

    //--- Helpers
    void initialize(const rapt::WorldCollisionHandler::PrimitiveList& worldCollisionPrimitives);

    //--- Evaluation
    double computeValue(const Eigen::VectorXd& q) const override;
    void computeGradient(Eigen::VectorXd& pVpQ, const Eigen::VectorXd& q) const override;
    void computeHessian(Eigen::SparseMatrixD& p2VpQ2, const Eigen::VectorXd& q) const override;

    //--- Tests
    bool testIndividualFirstDerivatives(const Eigen::VectorXd& q) const override;
    bool testIndividualSecondDerivatives(const Eigen::VectorXd& q) const override;
    bool testGradient(const Eigen::VectorXd& x) const override;
    bool testHessian(const Eigen::VectorXd& x) const override;

    //--- Solver
    bool preValueEvaluation(const Eigen::VectorXd& q) const override;
    void preDerivativeEvaluation(const Eigen::VectorXd& q) const override;

    //--- Constraints
    bool checkConstraintSatisfaction(const Eigen::VectorXd& q) const override;

    //--- Gui
    void drawGui() override;

public:
    std::vector<samp::TotalObjective> totalSAMPObjectives;
    const MotionTrajectoryHandler& trajectoryHandler;
};

}  // namespace lenny::mamp