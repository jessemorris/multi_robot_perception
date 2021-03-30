#ifndef _ROS_VDO_SCENE_GRAPH_BUILDER
#define _ROS_VDO_SCENE_GRAPH_BUILDER

#include <ros/ros.h>
#include <vector>
#include <realtime_vdo_slam/VdoSlamScene.h>
#include <sensor_msgs/CameraInfo.h>

#include "scene_graph/SceneGraph.hpp"
#include "visualizer/RosVisualizer.hpp"
#include "data_provider/RosBagDataProvider.hpp"

#include "CameraInformation.hpp"


namespace VDO_SLAM {

    class SceneGraphBuilder {
        
        public:
            SceneGraphBuilder();

            void slam_scene_callback(const realtime_vdo_slam::VdoSlamSceneConstPtr& scene);
            void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camera_info_msg);
            void load_data();
            void construct_graph();

        private:
            ros::NodeHandle nh;
            RosBagDataProviderPtr data_provider;

            ros::Publisher scene_pub;
            RosVisualizerPtr ros_viz;
            RosVizualizerSpinHandler ros_viz_handler;


            std::vector<realtime_vdo_slam::VdoSlamSceneConstPtr> slam_scenes;
            SceneGraphPtr graph;

            CameraInformationPtr camera_info;


    };

}

#endif