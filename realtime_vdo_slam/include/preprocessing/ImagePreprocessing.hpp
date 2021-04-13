#ifndef _ROS_VDO_IMAGE_PREPROCESSING
#define _ROS_VDO_IMAGE_PREPROCESSING



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

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



// #incl
#include <mono_depth_2/MonoDepthInterface.hpp>
#include <mask_rcnn/MaskRcnnInterface.hpp>
#include <flow_net/FlowNetInterface.hpp>
#include <midas_ros/MidasDepthInterface.hpp>

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

#define PROCESSING_INTERFACES_HPP mono_depth_2::MonoDepthInterfacePtr& mono_ptr, \
            mask_rcnn::MaskRcnnInterfacePtr& mask_ptr, \
            flow_net::FlowNetInterfacePtr& flow_ptr

#define PROCESSING_INTERFACES mono_depth_2::MonoDepthInterfacePtr& mono_ptr, \
    mask_rcnn::MaskRcnnInterfacePtr& mask_ptr, \
    flow_net::FlowNetInterfacePtr& flow_ptr

#define INIT_INTERFACES mono_ptr, mask_ptr, flow_ptr


namespace VDO_SLAM {
    

    namespace preprocessing {

        typedef message_filters::Subscriber<sensor_msgs::Image> MessageFilterSubscriber;
        typedef std::shared_ptr<MessageFilterSubscriber> MessageFilterSubscriberPtr;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ApproxRgbDepthSynchPolicy;
        typedef message_filters::Synchronizer<ApproxRgbDepthSynchPolicy> ApproxRgbDepthSynch;
        typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> RgbDepthSynch;
        typedef std::shared_ptr<RgbDepthSynch> RgbDepthSynchPtr;
        typedef std::shared_ptr<ApproxRgbDepthSynch> ApproxRgbDepthSynchPtr;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproxRgbDepthSegSynchPolicy;
        typedef message_filters::Synchronizer<ApproxRgbDepthSegSynchPolicy> ApproxRgbDepthSegSynch;
        typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> RgbDepthSegSynch;
        typedef std::shared_ptr<RgbDepthSegSynch> RgbDepthSegSynchPtr;
        typedef std::shared_ptr<ApproxRgbDepthSegSynch> ApproxRgbDepthSegSynchPtr;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproxAllSynchPolicy;
        typedef message_filters::Synchronizer<ApproxAllSynchPolicy> ApproxAllSynch;
        typedef message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> AllSynch;
        typedef std::shared_ptr<AllSynch> AllSynchPtr;
        typedef std::shared_ptr<ApproxAllSynch> ApproxAllSynchPtr;

        typedef const sensor_msgs::ImageConstPtr ImageConstPtr;

        enum InputType {
            RGB = 0,
            RGB_DEPTH = 1,
            RGB_DEPTH_SEG = 2,
            RGB_DEPTH_SEG_FLOW = 3,
            INVALID = 4
        };


        
        class BaseProcessing {

        public:
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


            static InputType get_input_type(ros::NodeHandle& n);
            virtual void start_services() {}


            

        protected:
            BaseProcessing(ros::NodeHandle& n, PROCESSING_INTERFACES_HPP);
            ~BaseProcessing();

            inline bool using_rgb_topic();
            inline bool using_depth_topic();
            inline bool using_seg_topic();
            inline bool using_flow_topic();

            std::string rgb_topic;
            std::string camera_info_topic;
            std::string depth_topic;
            std::string seg_topic;
            std::string flow_topic;
            std::string rgb_info;

            InputType input_type;

            // either rgbd or rgb
            eSensor camera_type;


            ros::NodeHandle handler;
            flow_net::FlowNetInterfacePtr sceneflow;
            mask_rcnn::MaskRcnnInterfacePtr mask_rcnn_interface;
            mono_depth_2::MonoDepthInterfacePtr mono_depth;
            midas_ros::MidasDepthInterfacePtr midas_depth;

            bool run_scene_flow = false; 
            bool run_mask_rcnn = false; 
            bool run_mono_depth = false;

            bool scene_flow_success;
            bool mask_rcnn_success;
            bool mono_depth_success;

            cv::Mat scene_flow_mat;
            cv::Mat scene_flow_viz;

            cv::Mat mask_rcnn_mat;
            cv::Mat mask_rcnn_viz;
            
            cv::Mat mono_depth_mat;

            image_transport::ImageTransport image_transport;


            image_transport::Subscriber rgb_subscriber;
            image_transport::Publisher rgb_repub;

            image_transport::Publisher maskrcnn_raw;
            image_transport::Publisher maskrcnn_viz;

            image_transport::Publisher flownet_raw;
            image_transport::Publisher flownet_viz;

            image_transport::Publisher monodepth_raw;

            ros::Publisher vdo_input_pub; //will publish realtime_vdo_slam::VdoInput msgs

            bool is_first;
            cv::Mat previous_image;

            
            //if set to true cv::undistort will be applied to the images
            bool undistord_images;

            CameraInformationPtr camera_info;


            ros::Time previous_time;
            ros::Time current_time;


        };

        class ImageRgb : public BaseProcessing {

            public:
                ImageRgb(ros::NodeHandle& nh_, PROCESSING_INTERFACES_HPP);

                void image_callback(ImageConstPtr& rgb);
                void start_services() override;

            protected:
                MessageFilterSubscriberPtr rgb_subscriber_synch;


        };

        class ImageRgbDepth : public ImageRgb {

            public:
                ImageRgbDepth(ros::NodeHandle& nh_, PROCESSING_INTERFACES_HPP);

                void image_callback(ImageConstPtr& rgb, ImageConstPtr& depth);
                void start_services() override;

            protected:
                MessageFilterSubscriberPtr depth_subscriber_synch;
            private:
                // RgbDepthSynchPtr rgb_depth_synch;
                ApproxRgbDepthSynchPtr rgb_depth_synch_ptr;

        };

        class ImageRgbDepthSeg : public ImageRgbDepth {

            public:
                ImageRgbDepthSeg(ros::NodeHandle& nh_, PROCESSING_INTERFACES_HPP);

                void image_callback(ImageConstPtr& rgb, ImageConstPtr& depth, ImageConstPtr& seg);
                void start_services() override;

            protected:
                MessageFilterSubscriberPtr seg_subscriber_synch;
            private:
                RgbDepthSegSynchPtr rgb_depth_seg_synch;


        };

        class ImageAll : public ImageRgbDepthSeg {

            public:
                ImageAll(ros::NodeHandle& nh_, PROCESSING_INTERFACES_HPP);

                void image_callback(ImageConstPtr& rgb, ImageConstPtr& depth, ImageConstPtr& seg, ImageConstPtr& flow);
                void start_services() override;

            protected:
                MessageFilterSubscriberPtr flow_subscriber_synch;
            private:
                AllSynchPtr all_synch;

        };
        typedef std::shared_ptr<BaseProcessing> BaseProcessingPtr;
        typedef std::unique_ptr<BaseProcessing> BaseProcessingUniquePtr;

        typedef std::shared_ptr<ImageRgb> ImageRgbPtr;
        typedef std::unique_ptr<ImageRgb> ImageRgbUniquePtr;

        typedef std::shared_ptr<ImageRgbDepth> ImageRgbDepthPtr;
        typedef std::unique_ptr<ImageRgbDepth> ImageRgbDepthUniquePtr;

        typedef std::shared_ptr<ImageRgbDepthSeg> ImageRgbDepthSegPtr;
        typedef std::unique_ptr<ImageRgbDepthSeg> ImageRgbDepthSegUniquePtr;

        typedef std::shared_ptr<ImageAll> ImageAllPtr;
        typedef std::unique_ptr<ImageAll> ImageAllUniquePtr;

    };

};


#endif