#ifndef _REALTIME_VDO_SLAM
#define _REALTIME_VDO_SLAM


#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>


#include <vdo_slam.hpp>


#include "SceneFlow.hpp"
#include "MaskRcnnInterface.hpp"
#include "MonoDepth.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <memory>

//wrapper for camera information
struct CameraInformation {
    std::string topic;
    sensor_msgs::CameraInfo camera_info;

};


class RealTimeVdoSLAM {

    public:
        RealTimeVdoSLAM(ros::NodeHandle& n);
        ~RealTimeVdoSLAM();


    private:
        std::string topic_prefix;
        std::string camera_suffix;
        std::string info_msg_suffix;

        std::string camera_selection;

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
        cv::Mat mask_rcnn_mat;
        cv::Mat mono_depth_mat;


        //evnetually become list when i have more than one camera
        std::string output_video_topic;
        std::string camea_info_topic;
        CameraInformation camera_information;
        

        image_transport::ImageTransport image_transport;
        image_transport::Subscriber image_subscriber;
        image_transport::Publisher maskrcnn_results;
        image_transport::Publisher flownet_results;
        image_transport::Publisher monodepth_results;

        bool is_first;
        cv::Mat previous_image;

        
        //if set to true cv::undistort will be applied to the images
        bool undistord_images;

        void image_callback(const sensor_msgs::ImageConstPtr& msg);

        //trajectory image for display
        cv::Mat image_trajectory;


        //VdoSlam
        std::unique_ptr<VDO_SLAM::System> slam_system;
        ros::Time previous_time;
        ros::Time current_time;



};



#endif