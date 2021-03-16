#ifndef _ROS_VDO_VISUALIZER
#define _ROS_VDO_VISUALIZER

#include <mutex>

#include <ros/ros.h>
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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>

namespace VDO_SLAM {

    /**
     * @brief Overlays an image with all information from a Slam Scene message.
     * This includes bounding box, class, tracking ID, pose and velocity
     * 
     * @param image rgb image to overlay
     * @param slam_scene slam scene to details
     * @return cv::Mat overlayed image
     */
    cv::Mat overlay_scene_image(const cv::Mat& image, const realtime_vdo_slam::VdoSlamScenePtr& slam_scene);


    class RosVisuazlier {

        public:
            RosVisuazlier(ros::NodeHandle& _nh);

            /**
             * @brief Subscribes to nav_msgs::Odometry messages that will be used for ground truth. Listenes to the /ros_vdo/odometry_ground_truth_topic
             * defined in the launch file. (see VDO_SLAM::Utils in RosScene.cpp) It will offset the odom gt bt some amount 
             * such that the gt odom starts at [0,0,0][0,0,0,1] even if the ROS information playing does not start centerd at the world 
             * coordinates. This purpose of this is to start the VDO_SLAM odom and the gt odom at the same place so we can compare them visually.
             * 
             * If odom started at the world frame, this would not be necessary as the VDO_SLAM odom starts at [0,0,0][0,0,0,1].
             * Internally, this will add the gt square to the display mat object (or should this be done by a function call?)
             * 
             * @param msg 
             */
            void odom_gt_callback(const nav_msgs::OdometryConstPtr& msg);


        private:

            ros::NodeHandle nh;
            ros::Publisher visualiser;
            ros::Publisher odom_pub;
            ros::Subscriber odom_repub_sub;
            cv::Mat display;
            std::mutex display_mutex;

            tf2_ros::TransformBroadcaster broadcaster;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener listener;

            nav_msgs::Odometry gt_odom;

            const int x_offset = 150;
            const int y_offset = 150;
            const int scale = 6;

            //Info needed for gt odom
            bool is_first;
            float odom_x_offset;
            float odom_y_offset;
            

        

            static int vis_count; //used for marker visualisation
    };

}


#endif