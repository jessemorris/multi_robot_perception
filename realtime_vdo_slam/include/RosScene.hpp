#ifndef _ROS_VDO_SLAM_SCENE
#define _ROS_VDO_SLAM_SCENE

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

#include <vdo_slam/vdo_slam.hpp>

namespace VDO_SLAM {

    /**
     * @brief ROS wrapper for the VDO_SLAM::SceneObject (found in the VDO_SLAM module).
     * 
     * Takes all key information and adds ros versions (if necessary) of each data. Includes functions to convert
     * to and from realtime_vdo_slam::VdoSceneObject msgs's so the information can be broadcast. 
     * 
     */
    class RosSceneObject: public SceneObject {

        public:
            /**
             * @brief Construct a new Ros Scene Object object from a realtime_vdo_slam::VdoSceneObject ROS msg.
             * 
             * @param _msg 
             */
            RosSceneObject(realtime_vdo_slam::VdoSceneObjectConstPtr& _msg);

            /**
             * @brief Construct a new Ros Scene Object object a VDO_SLAM::SceneObject, a ros::Time and
             * unique scene ID. The ros::Time object will come from the VDO_SLAM::RosScene object and synchronize all objects
             * to a sinqle scene (and also a unique scene id).
             * 
             * @param _object 
             * @param _time 
             * @param _uid 
             */
            RosSceneObject(SceneObject& _object,  ros::Time& _time, int _uid);

            /**
             * @brief Converts the object to a realtime_vdo_slam::VdoSceneObject so it can be broadcast. 
             * 
             * @return realtime_vdo_slam::VdoSceneObjectPtr 
             */
            realtime_vdo_slam::VdoSceneObjectPtr to_msg();

            ros::Time time;
            int uid;
    };

    
    class RosScene : public Scene {


        /**
         * @brief ROS wrapper for the VDO_SLAM::Scene object. This encapsulates all data for a single frame and
         * inluces the camera translation, rotation (as defined by the Scene object) as well as a list of 
         * VDO_SLAM::RosSceneObject's.
         * 
         */

        public:
            /**
             * @brief Construct a new Ros Scene object from a realtime_vdo_slam::VdoSlamScene msg. 
             * 
             * @param _msg 
             */
            RosScene(realtime_vdo_slam::VdoSlamSceneConstPtr& _msg);

            /**
             * @brief Construct a new Ros Scene object from a VDO_SLAM::Scene object and a ros::Time.
             * 
             * @param _object 
             * @param _time
             * @param _frame_id The frame with which to plot relative too. Usually the odom frame.
             * @param _child_frame_id The frame which the scene is in. Usually base_link/camera_link equivalent.
             */
            RosScene(Scene& _object, ros::Time _time, std::string& _frame_id, std::string& _child_frame_id);

            /**
             * @brief Gets the odometry of the camera as determined by the camera translation and rotation matrix.
             * 
             * @return const nav_msgs::Odometry& 
             */
            const nav_msgs::Odometry& odom_msg() const;

            /**
             * @brief Gets the transform stamped messaged realtive to the vdo_slam world frame.
             * 
             * @return const geometry_msgs::TransformStamped& 
             */
            const geometry_msgs::TransformStamped& tf_transform_msg() const;

            /**
             * @brief Constructs a visualisation in ROS of all the objects in the scene realtive to the vdo_slam world frame.
             * 
             * @param marker_array 
             */
            void make_vizualisation(visualization_msgs::MarkerArray& marker_array);

            /**
             * @brief Converts the object to a ROS msg so it can be broadcast. 
             * 
             * @return realtime_vdo_slam::VdoSlamScenePtr 
             */
            realtime_vdo_slam::VdoSlamScenePtr to_msg();

            const std::string& get_frame_id();
            const std::string& get_child_frame_id();
            const ros::Time& get_ros_time();

        private:
            ros::Time time;

            nav_msgs::Odometry odom;
            geometry_msgs::TransformStamped transform_stamped;

            std::string frame_id;
            std::string child_frame_id;

        
    };

};

typedef std::unique_ptr<VDO_SLAM::RosScene> RosScenePtr;

#endif
