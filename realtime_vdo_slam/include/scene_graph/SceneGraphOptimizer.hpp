#ifndef VDO_ROS_SCENE_GRAPH_OPTIMIZER
#define VDO_ROS_SCENE_GRAPH_OPTIMIZER


#include <minisam/core/Eigen.h>
#include <minisam/core/Factor.h>
#include <minisam/core/FactorGraph.h>
#include <minisam/core/LossFunction.h>
#include <minisam/core/Variables.h>
#include <minisam/nonlinear/LevenbergMarquardtOptimizer.h>

class ExpCurveFittingFactor : public minisam::Factor {
    private:
        Eigen::Vector2d p_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ExpCurveFittingFactor(minisam::Key key, const Eigen::Vector2d& point,
                            const std::shared_ptr<minisam::LossFunction>& lossfunc);
                            
        virtual ~ExpCurveFittingFactor() = default;

        // deep copy function
        std::shared_ptr<Factor> copy() const;

        // error function
        // clang-format off
        Eigen::VectorXd error(const minisam::Variables& values) const override;

        // jacobians function
        std::vector<Eigen::MatrixXd> jacobians(const minisam::Variables& values) const;
        // clang-format on
};


#endif
