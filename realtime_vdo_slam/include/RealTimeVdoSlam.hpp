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

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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
#include <queue>
#include <mutex>
#include <thread>


namespace VDO_SLAM {

    class RosSceneObject: public SceneObject {

        public:
            RosSceneObject(SceneObject& _object,  ros::Time& _time, int _uid);
            ros::Time time;
            int uid;
    };
    
    class RosScene : public Scene {

        public:
            RosScene(ros::NodeHandle& _nh, Scene& _object, ros::Time _time = ros::Time::now());
            void display_scene();

        private:
            std::string child_frame_id;
            ros::Time time;
            ros::NodeHandle nh;
            ros::Publisher visualiser;
            ros::Publisher odom_pub;
            tf2_ros::TransformBroadcaster broadcaster;
        
    };


};

//wrapper for camera information
struct CameraInformation {
    std::string topic;
    sensor_msgs::CameraInfo camera_info;

};

struct VdoSlamInput {
    cv::Mat raw, flow, depth, mask;
    std::vector<std::vector<float> > object_pose_gt;
    cv::Mat ground_truth;
    double time_diff;

    VdoSlamInput(cv::Mat& _raw, cv::Mat& _flow, cv::Mat& _depth, cv::Mat& _mask, double _time_diff) : 
        raw(_raw),
        flow(_flow),
        time_diff(_time_diff)

    {
        ground_truth = cv::Mat::eye(4,4,CV_32F);
        _depth.convertTo(depth, CV_32F);
        _mask.convertTo(mask, CV_32SC1);
    }

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

        int global_optim_trigger;

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
        void set_scene_labels(VDO_SLAM::Scene& scene);
        void vdo_worker();

        std::shared_ptr<VdoSlamInput> pop_vdo_input();
        void push_vdo_input(std::shared_ptr<VdoSlamInput> input);

        //trajectory image for display
        cv::Mat image_trajectory;


        //VdoSlam
        std::unique_ptr<VDO_SLAM::System> slam_system;
        std::unique_ptr<VDO_SLAM::RosScene> ros_scene;
        std::queue<std::shared_ptr<VdoSlamInput>> vdo_input_queue;
        std::mutex queue_mutex;
        std::thread vdo_worker_thread;
        ros::Time previous_time;
        ros::Time current_time;



};



#endif