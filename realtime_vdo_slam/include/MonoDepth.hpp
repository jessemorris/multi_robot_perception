#ifndef _REALTIME_VDOSLAM_MONODEPTH
#define _REALTIME_VDOSLAM_MONODEPTH

// ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <flow_net/FlowNet.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>

class MonoDepth {

    public:
        MonoDepth(ros::NodeHandle& n);
        ~MonoDepth() {};


        bool analyse_image(cv::Mat& current_image, cv::Mat& dst);
        bool start_service();
        static bool wait_for_mono_services(ros::Duration timeout = ros::Duration(-1));

    private:
        ros::NodeHandle nh;

        //the actual call to the monp depth network
        ros::ServiceClient mono_depth_client;

        ros::ServiceClient mono_depth_start;
        bool service_started;



        

};













#endif
