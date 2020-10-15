#ifndef _REALTIME_VDOSLAM_SCENE_FLOW
#define _REALTIME_VDOSLAM_SCENE_FLOW

// ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>

class SceneFlow {

    public:
        SceneFlow() {};
        ~SceneFlow() {};


        void analyse_image(cv::Mat& src, cv::Mat& dst);


        

};













#endif