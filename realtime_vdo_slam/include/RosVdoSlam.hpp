#ifndef _REALTIME_VDO_SLAM
#define _REALTIME_VDO_SLAM


#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>


#include <vdo_slam/vdo_slam.hpp>


#include <mask_rcnn/MaskRcnnInterface.hpp>
#include "RosScene.hpp"
#include "RosSceneManager.hpp"
#include "VdoSlamInput.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <memory>
#include <queue>
#include <mutex>
#include <thread>



typedef const sensor_msgs::ImageConstPtr& ImageConst;


class RosVdoSlam {

    public:
        /**
         * @brief Runs the Vdo Slam algorithm in the ROS environment. VDO requires input as RGB image, semantic mask,
         * optical flow and depth map. The class synchronizes the input messages using message_filter::TimeSynchronizer
         * and parses the images to the VDO_SLAM::System class. As the time for computation is significantly greater
         * than the frequency of any reayltime data stream a threaded queue is used to parse input VdoSlamInput to the 
         * VDO_SLAM::System. 
         * 
         * @param n 
         */
        RosVdoSlam(ros::NodeHandle& n);
        ~RosVdoSlam();

        /**
         * @brief Synchronized message callback to obtain the data needed for the the VDO_SLAM::System. The callback listens 
         * to the topics. Becuase this information comes from the vdo_slam_preprocesing node (which uses the parent namespace /vdoslam/input/)
         * we remap these topic to the desired topics here (see the launch file).
         * - raw_image: /camera/rgb/image_raw
         * - mask: /camera/mask/image_raw
         * - flow: /camera/flow/image_raw
         * - depth /camera/depth/image_raw
         * 
         * This callback creates as pointer to a VdoSlamInput which contains all the input necessary to the VDO_SLAM algorithm and adds 
         * it to the worker queue. 
         * 
         * @param raw_image 
         * @param mask 
         * @param flow 
         * @param depth 
         */
        void vdo_input_callback(ImageConst raw_image, ImageConst mask, ImageConst flow, ImageConst depth);

    private:
        ros::NodeHandle handle;
        
        //we need this to request labels TODO: take out dependancy
        mask_rcnn::MaskRcnnInterface mask_rcnn_interface;

        /**
         * @brief Set the scene labels (from the index semantic index of each object) for a particular scene.
         * 
         * Uses the mask_rcnn::MaskRcnnInterface to query the categories list and then sets the label for each object
         * which has the associated index. This is required becuase the VDO_SLAM::Scene object does not have access to the 
         * direct label (in order to keep the VDO_SLAM module separate to the mask_rcnn interface). Once the scene has been generated
         * we must use this function to set the labels (as each image has the index semantic label).
         * 
         * @param scene 
         */
        void set_scene_labels(std::unique_ptr<VDO_SLAM::Scene>& scene);


        /**
         * @brief Constructs the desired slam system with parameters and configuration defined
         * in the launch file.
         * 
         * @param nh Ros::NodeHandle
         * @return std::shared_ptr<VDO_SLAM::System>
         */
        std::shared_ptr<VDO_SLAM::System> construct_slam_system(ros::NodeHandle& nh);


        /**
         * @brief Worker thread for the VDO_SLAM queue. Will continue as long as ros::okay() returns true. 
         * 
         */
        void vdo_worker();

        /**
         * @brief Gets the latest input for the VDO_SLAM algorithm. This call can happen asynchronisly as the VDO algorithm
         * will run at a different rate to the input. 
         * 
         * @return std::shared_ptr<VdoSlamInput> 
         */
        std::shared_ptr<VDO_SLAM::VdoSlamInput> pop_vdo_input();

        /**
         * @brief Adds a new input for the VDO_SLAM algorithm. This call can happen asynchronisly as the VDO algorithm
         * will run at a different rate to the input and is called from the callback function
         * 
         * @param input 
         */
        void push_vdo_input(std::shared_ptr<VDO_SLAM::VdoSlamInput>& input);


        int global_optim_trigger;

        //Frames
        std::string odom_frame_id;
        std::string base_link_frame_id;
        std::string map_frame_id;


        //VdoSlam
        VDO_SLAM::RosSceneManager ros_scene_manager;
        std::unique_ptr<VDO_SLAM::RosScene> ros_scene;

        cv::Mat image_trajectory;
        std::shared_ptr<VDO_SLAM::System> slam_system;

        //VdoSlam input
        std::queue<std::shared_ptr<VDO_SLAM::VdoSlamInput>> vdo_input_queue;
        std::mutex queue_mutex;
        std::thread vdo_worker_thread;

        //data synchronizers
        message_filters::Subscriber<sensor_msgs::Image> raw_img;
        message_filters::Subscriber<sensor_msgs::Image> mask_img;
        message_filters::Subscriber<sensor_msgs::Image> flow_img;
        message_filters::Subscriber<sensor_msgs::Image> depth_img;

        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync;

        ros::Time current_time;
        ros::Time previous_time;


};



#endif