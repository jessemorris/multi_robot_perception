#ifndef _ROS_VDO_SCENE_GRAPHS_DATA_PROVIDER
#define _ROS_VDO_SCENE_GRAPHS_DATA_PROVIDER

#include <ros/ros.h>


#include <functional>
#include <string>

#include <opencv2/opencv.hpp>

#include <image_transport/subscriber_filter.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <realtime_vdo_slam/VdoSlamScene.h>
#include <sensor_msgs/CameraInfo.h>
#include "CameraInformation.hpp"
#include <future>
#include <functional>


namespace VDO_SLAM {
    
    typedef std::function<void(const realtime_vdo_slam::VdoSlamSceneConstPtr&)> SlamSceneCallbackFunc;
    typedef std::function<void(const sensor_msgs::CameraInfoConstPtr&)> CameraInfoCallbackFunc;

    class DataProviderInterface {


        public:
            DataProviderInterface();

            virtual ~DataProviderInterface() {}
            virtual bool spin() = 0;

            void connect_slam_scene_callback(SlamSceneCallbackFunc&& func);
            void connect_camera_info_callback(CameraInfoCallbackFunc&& func);

        protected:
            SlamSceneCallbackFunc slam_scene_callback;
            bool slam_scene_connected = false;

            CameraInfoCallbackFunc camera_info_callback;
            bool camera_info_connected = false;


            ros::NodeHandle nh;
            CameraInformation camer_info;

    };
}


#endif