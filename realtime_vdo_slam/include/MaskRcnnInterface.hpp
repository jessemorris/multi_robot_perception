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


        bool analyse_image(cv::Mat& current_image, cv::Mat& dst, std::vector<std::string>& labels, std::vector<int>& label_indexs);
        bool start_service();

        static bool request_labels(const std::vector<int>& label_indexs, std::vector<std::string>& labels);
        static std::string& request_label(int index);
        static bool set_mask_labels(ros::NodeHandle& nh, ros::Duration timeout = ros::Duration(-1));

    private:
        ros::NodeHandle nh;
        ros::ServiceClient mask_rcnn_client;
        ros::ServiceClient mask_rcnn_start;


        
        static bool labels_found;
        static std::vector<std::string> mask_labels; //we will request this at the start of the program from the mask rcnn interface
        static std::string invalid_name;

        bool service_started;



        

};


#endif
