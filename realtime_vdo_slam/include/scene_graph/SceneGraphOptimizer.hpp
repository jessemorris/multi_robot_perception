#ifndef VDO_ROS_SCENE_GRAPH_OPTIMIZER
#define VDO_ROS_SCENE_GRAPH_OPTIMIZER


#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

#include <iostream>


class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        virtual void setToOriginImpl();

        virtual void oplusImpl(const double* update);

        virtual bool read(std::istream& in ) {}
        virtual bool write(std::ostream& out ) const {}
};

class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        CurveFittingEdge(double x);
        
        void computeError();

        virtual bool read(std::istream& in ) {}
        virtual bool write(std::ostream& out ) const {}

        public:
            double _x;
};

#endif
