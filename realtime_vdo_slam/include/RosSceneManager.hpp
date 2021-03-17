#ifndef _ROS_VDO_SLAM_SCENE_MANAGER_HPP
#define _ROS_VDO_SLAM_SCENE_MANAGER_HPP

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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mutex>

#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>

#include "RosScene.hpp"

namespace VDO_SLAM {


    class RosSceneManager {
        
        /**
         * @brief Manager to control and visualise a VDO_SLAM scene. To simply the construction of each scene, the manager holds 
         * all the visualisation tools (ros::publishers and tf::listeners etc) and can then update the visualisation graphs/odom/transforms
         * etc.
         * 
         */
        public:
            /**
             * @brief Construct a new Ros Scene Manager object
             * 
             * @param _nh 
             */
            RosSceneManager(ros::NodeHandle& _nh);

            /**
             * @brief Updates the ROS related information for this scene including the camera odometry, marker array of all the objects
             * within the scene and the transform tree.
             * 
             * @param scene 
             */
            void display_scene(RosSceneUniquePtr& scene);

            /**
             * @brief Get the display mat object
             * 
             * @return cv::Mat& 
             */
            cv::Mat& get_display_mat();

            /**
             * @brief Subscribes to nav_msgs::Odometry messages that will be used for ground truth. Listenes to the /ros_vdo/odometry_ground_truth_topic
             * defined in the launch file. (see VDO_SLAM::Utils in RosScene.cpp) It will offset the odom gt bt some amount 
             * such that the gt odom starts at [0,0,0][0,0,0,1] even if the ROS information playing does not start centerd at the world 
             * coordinates. This purpose of this is to start the VDO_SLAM odom and the gt odom at the same place so we can compare them visually.
             * 
             * If odom started at the world frame, this would not be necessary as the VDO_SLAM odom starts at [0,0,0][0,0,0,1].
             * Intenrally, this will add the gt square to the display mat object.
             * 
             * @param msg 
             */
            void odom_gt_callback(const nav_msgs::OdometryConstPtr& msg);


            const nav_msgs::Odometry& get_latest_gt_odom();

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
