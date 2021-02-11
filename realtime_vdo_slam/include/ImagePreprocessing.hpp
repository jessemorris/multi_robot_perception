#ifndef _VDO_IMAGE_PREPROCESSING
#define _VDO_IMAGE_PREPROCESSING



#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>



#include "SceneFlow.hpp"
#include "MaskRcnnInterface.hpp"
#include "MonoDepth.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>


//wrapper for camera information
struct CameraInformation {
    typedef std::unique_ptr<CameraInformation> CameraInformationPtr;

    std::string topic;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat map1, map2, modified_camera_matrix;
    sensor_msgs::CameraInfo camera_info_msg;

};

namespace VDO_SLAM {

    class ImagePrepcoessing {

    public:
        ImagePrepcoessing(ros::NodeHandle& n);
        ~ImagePrepcoessing();

        void undistortImage(cv::Mat& input, cv::Mat& undistorted);


    private:
        std::string input_camera_topic;
        // // std::string topic_prefix;
        // // std::string camera_suffix;
        // // std::string info_msg_suffix;

        // std::string camera_selection;

        ros::NodeHandle handler;
        SceneFlow sceneflow;
        MaskRcnnInterface mask_rcnn_interface;
        MonoDepth mono_depth;

        bool run_scene_flow;
        bool run_mask_rcnn;
        bool run_mono_depth;

        bool scene_flow_success;
        bool mask_rcnn_success;
        bool mono_depth_success;

        cv::Mat scene_flow_mat;
        cv::Mat scene_flow_viz;

        cv::Mat mask_rcnn_mat;
        cv::Mat mask_rcnn_viz;
        
        cv::Mat mono_depth_mat;



        image_transport::ImageTransport image_transport;
        image_transport::Subscriber image_subscriber;

        image_transport::Publisher input_image;

        image_transport::Publisher maskrcnn_raw;
        image_transport::Publisher maskrcnn_viz;

        image_transport::Publisher flownet_raw;
        image_transport::Publisher flownet_viz;

        image_transport::Publisher monodepth_raw;

        bool is_first;
        cv::Mat previous_image;

        
        //if set to true cv::undistort will be applied to the images
        bool undistord_images;

        void image_callback(const sensor_msgs::ImageConstPtr& msg);


        //trajectory image for display
        cv::Mat image_trajectory;

        std::string input_camera_info_topic;

        CameraInformation camera_info;


        ros::Time previous_time;
        ros::Time current_time;

        //for rectification purposes




    };

};


#endif