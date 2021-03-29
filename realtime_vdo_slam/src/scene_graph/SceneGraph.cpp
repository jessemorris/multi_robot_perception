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
#include <algorithm>  

#include <opencv2/opencv.hpp>

#include <ros/ros.h>

SceneGraph::SceneGraph() {
    ROS_INFO_STREAM("using Cauchy loss");
    loss = minisam::CauchyLoss::Cauchy(1.0);
}


void SceneGraph::add_dynamic_object(realtime_vdo_slam::VdoSlamScene& scene) {
    realtime_vdo_slam::VdoSlamScenePtr scene_ptr = boost::make_shared<realtime_vdo_slam::VdoSlamScene>(scene);
    slam_scenes.push_back(scene_ptr);

    for(realtime_vdo_slam::VdoSceneObject& object : scene.objects) {
        //TODO: currently not adding slam scene association to SlamObjectAssociation
        //first time we see this dynamic object
        if (dyn_object_map.find(object.tracking_id) == dyn_object_map.end() ) {
            SlamObjectAssociationVector vector;

            SlamObjectAssociation object_association;
            object_association.slam_scene_ptr = scene_ptr;
            object_association.object = object;
            object_association.time = object.time;

            vector.push_back(object_association);
            dyn_object_map.insert(std::make_pair(object.tracking_id, vector));
        }
        else {
            SlamObjectAssociation object_association;
            object_association.object = object;
            object_association.time = object.time;
            object_association.slam_scene_ptr = scene_ptr;
            dyn_object_map[object.tracking_id].push_back(object_association);
        }
    }
}

std::map<TrackingId, CurveParamPair> SceneGraph::optimize_object_poses() {
    std::map<TrackingId, CurveParamPair> optimized_map;

    //get data for each object
    DynObjectMap::iterator it;
    for (it = dyn_object_map.begin(); it != dyn_object_map.end(); it++) {
        ROS_INFO_STREAM("Collecting data for track id: " << it->first);
        minisam::FactorGraph factor_graph;


        for (SlamObjectAssociation& object_association : it->second) {
            factor_graph.add(ExpCurveFittingFactor(minisam::key('p', 0), Eigen::Vector2d(object_association.object.pose.position.x,
                                                object_association.object.pose.position.y), loss));

        }

        ROS_INFO_STREAM("Optimizing for " << it->second.size() << " data points");
        minisam::Variables init_values;
        init_values.add(minisam::key('p', 0), Eigen::Vector2d(0, 0));
        ROS_INFO_STREAM("initial curve parameters :"  << init_values.at<Eigen::Vector2d>(minisam::key('p', 0)));

        // optimize!
        minisam::LevenbergMarquardtOptimizerParams opt_param;
        opt_param.verbosity_level = minisam::NonlinearOptimizerVerbosityLevel::ITERATION;
        minisam::LevenbergMarquardtOptimizer opt(opt_param);

        minisam::Variables values;
        opt.optimize(factor_graph, init_values, values);
        ROS_INFO_STREAM("opitmized curve parameters :" << values.at<Eigen::Vector2d>(minisam::key('p', 0)));

        CurveParamPair pair;
        Eigen::Vector2d result = values.at<Eigen::Vector2d>(minisam::key('p', 0));
        pair.first = result[0]; //m
        pair.second = result[1]; //c
        optimized_map.insert({it->first, pair});
        
    }

    return optimized_map;

}

SlamSceneVector SceneGraph::reconstruct_slam_scene(std::map<TrackingId, CurveParamPair>& optimized_poses) {
    for(realtime_vdo_slam::VdoSlamScenePtr& scene_ptr : slam_scenes) {

        for(realtime_vdo_slam::VdoSceneObject& scene_object : scene_ptr->objects) {
            int track_id = scene_object.tracking_id;
            CurveParamPair& curve_params = optimized_poses[track_id];
            double m = curve_params.first;
            double c = curve_params.second;
            double smooth_y = exp(m * scene_object.pose.position.x + c);
            scene_object.pose.position.y = smooth_y;
        }
    }




}


void SceneGraph::show_optimized_poses(std::map<TrackingId, CurveParamPair>& optimized_poses, int random_samples) {
    //generate n random samples
    //no good way to do this so we make a list of size K (number of total tracking IDs), randomly shuffle and take the first n
    ROS_INFO_STREAM("Randomly sampling: " << random_samples);
    int size = dyn_object_map.size();
    ROS_INFO_STREAM(size);
    std::vector<int> samples(size);
    for (int i = 0; i < size; i++) {
        samples[i] = i;
    }

    std::random_shuffle(samples.begin(), samples.end());
    if (random_samples == -1) {
        random_samples = size;
    }

    cv::Mat display = cv::Mat::zeros(800, 800, CV_8UC3);

    //currently same code as in RosVisualizer -> coudl put into config file so all nodes can see
    const int x_offset = 150;
    const int y_offset = 150;
    const int scale = 6;

    //i will be the tracking ID
    for(int i = 0; i < random_samples; i++) {
        int track = samples[i];
        ROS_INFO_STREAM(track);

        SlamObjectAssociationVector& object_vector = dyn_object_map[track];
        CurveParamPair& curve_params = optimized_poses[track];
        double m = curve_params.first;
        double c = curve_params.second;

        for (SlamObjectAssociation& object_association : object_vector) {
            ROS_INFO_STREAM_ONCE("Plotting original path for track " << track << " and class " << object_association.object.label);
            double x = object_association.object.pose.position.x;
            double y = object_association.object.pose.position.y;

            //plot original path
            int x_display =  static_cast<int>(x*scale) + x_offset;
            int y_display =  static_cast<int>(y*scale) + y_offset;
            cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // orange

            //construct smooth path using y=exp(mx+c)
            ROS_INFO_STREAM(m << " " << c << " x " << x);
            double smooth_y = exp(m * x + c);
            y_display =  static_cast<int>(smooth_y*scale) + y_offset;
            cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // green

            cv::imshow("Optimized Poses", display);
            cv::waitKey(1);
        }

    }



    cv::imshow("Optimized Poses", display);
    cv::waitKey();


}