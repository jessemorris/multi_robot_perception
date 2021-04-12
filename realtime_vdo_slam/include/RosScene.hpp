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
#include <realtime_vdo_slam/VdoSlamMap.h>


#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

#include <vdo_slam/utils/Types.h>
#include <vdo_slam/Scene.h>

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
            RosSceneObject(realtime_vdo_slam::VdoSceneObject& _msg);

            /**
             * @brief Construct a new Ros Scene Object object a VDO_SLAM::SceneObject, a ros::Time and
             * unique scene ID. The ros::Time object will come from the VDO_SLAM::RosScene object and synchronize all objects
             * to a sinqle scene (and also a unique scene id).
             * 
             * @param _object 
             * @param _time 
             * @param _uid 
             */
            RosSceneObject(SceneObject& _object,  ros::Time& _time);

            /**
             * @brief Converts the object to a realtime_vdo_slam::VdoSceneObject so it can be broadcast. 
             * 
             * @return realtime_vdo_slam::VdoSceneObjectPtr 
             */
            realtime_vdo_slam::VdoSceneObjectPtr to_msg();

            friend std::ostream& operator<<(std::ostream& os, const RosSceneObject& scene);


            ros::Time time;
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
             * @brief Creates a slam map message from a vector of Scenes. This can then be sent to the visualizer for
             * map reconstruction
             * 
             * @param map std::vector<SlamScenePtr>& 
             * @return realtime_vdo_slam::VdoSlamMapConstPtr 
             */
            static realtime_vdo_slam::VdoSlamMapPtr make_map(std::vector<SlamScenePtr>& map);

            /**
             * @brief Gets the odometry of the camera as determined by the camera translation and rotation matrix.
             * 
             * @return const nav_msgs::Odometry& 
             */
            const nav_msgs::Odometry& odom_msg() const;

            /**
             * @brief Converts the object to a ROS msg so it can be broadcast. 
             * 
             * @return realtime_vdo_slam::VdoSlamScenePtr 
             */
            realtime_vdo_slam::VdoSlamScenePtr to_msg();

            /**
             * @brief Get the ros time object
             * 
             * @return const ros::Time& 
             */
            const ros::Time& get_ros_time();
            friend std::ostream& operator<<(std::ostream& os, const RosScene& scene);


        private:
            ros::Time time;
            nav_msgs::Odometry odom;
            cv::Mat rgb_image;



        
    };

};

typedef std::shared_ptr<VDO_SLAM::RosSceneObject> RosSceneObjectPtr;
typedef std::unique_ptr<VDO_SLAM::RosSceneObject> RosSceneObjectUniquePtr;

typedef std::shared_ptr<VDO_SLAM::RosScene> RosScenePtr;
typedef std::unique_ptr<VDO_SLAM::RosScene> RosSceneUniquePtr;

#endif
