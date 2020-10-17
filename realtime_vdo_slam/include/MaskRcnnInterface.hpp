#ifndef _REALTIME_VDOSLAM_MASKRCNN_INTERFACE
#define _REALTIME_VDOSLAM_MASKRCNN_INTERFACE

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

class MaskRcnnInterface {

    public:
        MaskRcnnInterface(ros::NodeHandle& n);
        ~MaskRcnnInterface() {};


        bool analyse_image(cv::Mat& current_image, cv::Mat& dst);

    private:
        ros::NodeHandle nh;
        ros::ServiceClient mask_rcnn_client;



        

};


#endif
