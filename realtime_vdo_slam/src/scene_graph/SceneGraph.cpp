#include "scene_graph/SceneGraph.hpp"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>


#include "scene_graph/SceneGraphOptimizer.hpp"

#include <ros/ros.h>

SceneGraph::SceneGraph() {
}


void SceneGraph::add_dynamic_object(realtime_vdo_slam::VdoSlamScene& scene) {

    for(realtime_vdo_slam::VdoSceneObject& object : scene.objects) {
        //TODO: currently not adding slam scene association to SlamObjectAssociation
        //first time we see this dynamic object
        if (dyn_object_map.find(object.tracking_id) == dyn_object_map.end() ) {
            SlamObjectAssociationVector vector;

            SlamObjectAssociation object_association;
            object_association.object = object;
            object_association.time = object.time;

            vector.push_back(object_association);
            dyn_object_map.insert(std::make_pair(object.tracking_id, vector));
        }
        else {
            SlamObjectAssociation object_association;
            object_association.object = object;
            object_association.time = object.time;
            dyn_object_map[object.tracking_id].push_back(object_association);
        }
    }
}

void SceneGraph::optimize_object_poses() {
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;
    std::unique_ptr<Block::LinearSolverType> linearSolver = std::make_unique<g2o::LinearSolverDense<Block::PoseMatrixType>>();
    std::unique_ptr<Block> solver_ptr = std::make_unique<Block>(std::move(linearSolver));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr );
    // g2o::OptimizationAlgorithmDogleg* solver = new g2o::OptimizationAlgorithmDogleg( solver_ptr );
    g2o::SparseOptimizer optimizer; 
    optimizer.setAlgorithm( solver );
    optimizer.setVerbose( true );

    CurveFittingVertex* v = new CurveFittingVertex();
    v->setEstimate( Eigen::Vector3d(0,0,0) );
    v->setId(0);
    optimizer.addVertex( v );

    const double w_sigma=1.0;

    //get data for each object
    DynObjectMap::iterator it;
    for (it = dyn_object_map.begin(); it != dyn_object_map.end(); it++) {
        ROS_INFO_STREAM("Collecting data for track id: " << it->first);
        std::vector<Eigen::Vector2d> data_points;

        int i = 0;
        for (SlamObjectAssociation& object_association : it->second) {
            CurveFittingEdge* edge = new CurveFittingEdge(object_association.object.pose.position.x);
            edge->setId(i);
            edge->setVertex(0, v);               
            edge->setMeasurement(object_association.object.pose.position.y);
            edge->setInformation(Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) );
            optimizer.addEdge( edge );
            
            i++;

        }

        ROS_INFO_STREAM("Optimizing for " << it->second.size() << " data points");
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        optimizer.initializeOptimization();
        optimizer.optimize(100);
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
        ROS_INFO_STREAM("solve time cost = "<<time_used.count()<<" seconds.");

        Eigen::Vector3d abc_estimate = v->estimate();
        ROS_INFO_STREAM("estimated model: "<<abc_estimate.transpose());

    }

}