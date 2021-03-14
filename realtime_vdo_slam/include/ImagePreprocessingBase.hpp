#ifndef _ROS_VDO_IMAGE_PREPROCESSING_BASE
#define _ROS_VDO_IMAGE_PREPROCESSING_BASE



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



// #incl
#include <mono_depth_2/MonoDepthInterface.hpp>
#include <mask_rcnn/MaskRcnnInterface.hpp>
#include <mask_rcnn/SemanticTracker.hpp>
#include <flow_net/FlowNetInterface.hpp>

#include <vdo_slam/System.h>

#include "CameraInformation.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>


namespace VDO_SLAM {
    

    eSensor camera_type_from_string(std::string& input);

    namespace preprocessing {

        /**
         * @brief ROS Class that pre-processes all data needed for the vdo-slam algorithm. A single image stream
         * is captured (see readtime_vdo.yaml for the input stream topic name) and a corresponding depth map, dense optical flow
         * and semantic mask is produced for each frame. This all happens within the callback and the
         * Neural Nets are called sequentially (I am sure you could paralellise this but I dont need to and my laptop has only 1 GPU!) 
         * 
         * These topics are then produced out (with synchronized time stamps)
         * with the parent namespace /vdoslam/input/camera. See readme for more information. Addition topics are produced to help with visualisation
         * and debugging. 
         * 
         * This interface between the Neural Networks (defined in the modules: mask_rcnn, flow_net, mono_depth_2) uses the Cpp
         * interface and so the python service starter must be running so that they correspdoning python nodes can be spun up.
         * 
         * Any image pre-prcoessing is done here (eg. image undistortion). The USyd Campus dataset requires images rectification (see 
         * undistort image paramater in realtime_vdo.yaml) and a sensor_msgs::camera_info topic stream namespace must be provided for this 
         * as well if True. 
         * 
         */
        class BaseProcessing {

        public:
            BaseProcessing(ros::NodeHandle& n);
            ~BaseProcessing();
            /**
             * @brief Undistorts the input image if undistort_image param is set to True. 
             * 
             * For the USyd campus data set have an equidistant model so we must first create a rectifcation 
             * map and then rectify based on the new camera matrix P. 
             * 
             * @param input 
             * @param undistorted 
             */
            void undistortImage(cv::Mat& input, cv::Mat& undistorted);
            // void image_callback(const sensor_msgs::ImageConstPtr& msg);

            

        protected:


            std::string input_camera_topic;

            //if param in launch file is set to rgbd
            std::string input_depth_camera_topic;

            // either rgbd or rgb
            eSensor camera_type;


            ros::NodeHandle handler;
            flow_net::FlowNetInterface sceneflow;
            mask_rcnn::MaskRcnnInterface mask_rcnn_interface;
            // mask_rcnn::SemanticTracker tracker;
            mono_depth_2::MonoDepthInterface mono_depth;

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



            //trajectory image for display
            cv::Mat image_trajectory;

            std::string input_camera_info_topic;
            std::string input_camera_depth_topic;

            CameraInformationPtr camera_info;


            ros::Time previous_time;
            ros::Time current_time;




        };

    };

};


#endif