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
             */
            RosScene(Scene& _object, ros::Time _time);

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

        private:
            ros::Time time;

            nav_msgs::Odometry odom;
            geometry_msgs::TransformStamped transform_stamped;

        
    };

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
            void display_scene(std::unique_ptr<VDO_SLAM::RosScene>& scene);

            /**
             * @brief Updates and visualises the birdseye view of the scene. It plots the camera pos as a red square and all tracked
             * 3D objects as coloured dots (currently coloured by classification). If gt odom is present it will also plot this as a green 
             * squares.
             * 
             * @param scene 
             */
            void update_display_mat(std::unique_ptr<VDO_SLAM::RosScene>& scene);

            /**
             * @brief Get the display mat object
             * 
             * @return cv::Mat& 
             */
            cv::Mat& get_display_mat();

            /**
             * @brief Subscribes to nav_msgs::Odometry messages that will be used for ground truth. Listenes to the /odom_repub topic
             *  (see VDO_SLAM::Utils in RosScene.cpp) that is essentially the gt odom (simply /odom) that is then offset by some amount 
             * such that the /odom_repub starts at [0,0,0][0,0,0,1] even if the ROS information playing does not start centerd at the world 
             * coordinates. This purpose of this is to start the VDO_SLAM odom and the gt odom at the same place so we can compare them visually.
             * 
             * If odom started at the world frame, this would not be necessary as the VDO_SLAM odom starts at [0,0,0][0,0,0,1].
             * Intenrally, this will add the gt square to the display mat object.
             * 
             * @param msg 
             */
            void odom_repub_callback(const nav_msgs::OdometryConstPtr& msg);


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
            std::string child_frame_id;

            nav_msgs::Odometry gt_odom;

            const int x_offset = 300;
            const int y_offset = 300;
            const int scale = 6;
            

        

            static int vis_count; //used for marker visualisation

    };


};

typedef std::unique_ptr<VDO_SLAM::RosScene> RosScenePtr;

#endif
