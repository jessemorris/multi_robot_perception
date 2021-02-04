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
            RosScene(ros::NodeHandle& _nh, Scene& _object, ros::Time _time);
            void display_scene();

        private:
            std::string child_frame_id;
            ros::Time time;
            ros::NodeHandle nh;
            ros::Publisher visualiser;
            ros::Publisher odom_pub;
            tf2_ros::TransformBroadcaster broadcaster;

            static int vis_count; //used for marker visualisation
        
    };


};
struct VdoSlamInput {
    cv::Mat raw, flow, depth, mask;
    std::vector<std::vector<float> > object_pose_gt;
    cv::Mat ground_truth;
    double time_diff;
    ros::Time image_time; //when the image was created so we can keep track of the real time despite algorithmic delays

    VdoSlamInput(cv::Mat& _raw, cv::Mat& _flow, cv::Mat& _depth, cv::Mat& _mask, double _time_diff, ros::Time& _image_time) : 
        raw(_raw),
        flow(_flow),
        time_diff(_time_diff),
        image_time(_image_time)

    {
        ground_truth = cv::Mat::eye(4,4,CV_32F);
        _depth.convertTo(depth, CV_32F);
        _mask.convertTo(mask, CV_32SC1);
    }

};

typedef const sensor_msgs::ImageConstPtr& ImageConst;


class RosVdoSlam {

    public:
        RosVdoSlam(ros::NodeHandle& n);
        ~RosVdoSlam();

        void vdo_input_callback(ImageConst raw_image, ImageConst mask, ImageConst flow, ImageConst depth);

    private:
        ros::NodeHandle handle;

        void set_scene_labels(std::unique_ptr<VDO_SLAM::Scene>& scene);
        void vdo_worker();

        std::shared_ptr<VdoSlamInput> pop_vdo_input();
        void push_vdo_input(std::shared_ptr<VdoSlamInput>& input);

        //ros time synchronizers for all input data

        int global_optim_trigger;
        //VdoSlam
        cv::Mat image_trajectory;
        std::unique_ptr<VDO_SLAM::System> slam_system;
        std::unique_ptr<VDO_SLAM::RosScene> ros_scene;
        std::queue<std::shared_ptr<VdoSlamInput>> vdo_input_queue;
        std::mutex queue_mutex;
        std::thread vdo_worker_thread;

        message_filters::Subscriber<sensor_msgs::Image> raw_img;
        message_filters::Subscriber<sensor_msgs::Image> mask_img;
        message_filters::Subscriber<sensor_msgs::Image> flow_img;
        message_filters::Subscriber<sensor_msgs::Image> depth_img;

        message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync;

        ros::Time current_time;
        ros::Time previous_time;


};



#endif