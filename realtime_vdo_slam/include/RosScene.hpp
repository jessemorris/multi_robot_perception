#ifndef _ROS_SCENE
#define _ROS_SCENE

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

#include <vdo_slam.hpp>

namespace VDO_SLAM {

    class RosSceneObject: public SceneObject {

        public:
            RosSceneObject(SceneObject& _object,  ros::Time& _time, int _uid);
            ros::Time time;
            int uid;
    };

    
    class RosScene : public Scene {


        public:
            RosScene(Scene& _object, ros::Time _time);

            const nav_msgs::Odometry& odom_msg() const;
            const geometry_msgs::TransformStamped& tf_transform_msg() const;
            void make_vizualisation(visualization_msgs::MarkerArray& marker_array);

        private:
            ros::Time time;

            nav_msgs::Odometry odom;
            geometry_msgs::TransformStamped transform_stamped;

        
    };

    class RosSceneManager {

        public:
            RosSceneManager(ros::NodeHandle& _nh);
            void display_scene(std::unique_ptr<VDO_SLAM::RosScene>& scene);
            void to_cv_mat(cv::Mat& display);

        private:
            ros::NodeHandle nh;
            ros::Publisher visualiser;
            ros::Publisher odom_pub;
            tf2_ros::TransformBroadcaster broadcaster;
            std::string child_frame_id;

        

            static int vis_count; //used for marker visualisation

    };


};

typedef std::unique_ptr<VDO_SLAM::RosScene> RosScenePtr;

#endif
