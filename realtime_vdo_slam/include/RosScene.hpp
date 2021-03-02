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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>




#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

#include <vdo_slam.hpp>

namespace VDO_SLAM {

    class RosSceneObject: public SceneObject {

        public:
            RosSceneObject(realtime_vdo_slam::VdoSceneObjectConstPtr& _msg);
            RosSceneObject(SceneObject& _object,  ros::Time& _time, int _uid);

            realtime_vdo_slam::VdoSceneObjectPtr to_msg();

            ros::Time time;
            int uid;
    };

    
    class RosScene : public Scene {


        public:
            RosScene(realtime_vdo_slam::VdoSlamSceneConstPtr& _msg);

            RosScene(Scene& _object, ros::Time _time);

            const nav_msgs::Odometry& odom_msg() const;
            const geometry_msgs::TransformStamped& tf_transform_msg() const;
            void make_vizualisation(visualization_msgs::MarkerArray& marker_array);

            realtime_vdo_slam::VdoSlamScenePtr to_msg();

        private:
            ros::Time time;

            nav_msgs::Odometry odom;
            geometry_msgs::TransformStamped transform_stamped;

        
    };

    class RosSceneManager {

        public:
            RosSceneManager(ros::NodeHandle& _nh);
            void display_scene(std::unique_ptr<VDO_SLAM::RosScene>& scene);
            void update_display_mat(std::unique_ptr<VDO_SLAM::RosScene>& scene);
            cv::Mat& get_display_mat();

            void odom_repub_callback(const nav_msgs::OdometryConstPtr& msg);

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
            std::string child_frame_id;

            const int x_offset = 300;
            const int y_offset = 300;
            const int scale = 3;

        

            static int vis_count; //used for marker visualisation

    };


};

typedef std::unique_ptr<VDO_SLAM::RosScene> RosScenePtr;

#endif
