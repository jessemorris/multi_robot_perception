#ifndef _VDO_ROS_UTILS
#define _VDO_ROS_UTILS

#include <string>

#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

namespace VDO_SLAM {

    namespace utils {

        void publish_static_tf(const nav_msgs::Odometry& odom,
                                const std::string& parent_frame_id,
                                const std::string& child_frame_id, const ros::Time& time = ros::Time::now());

        void odom_to_tf(const nav_msgs::Odometry& odom, geometry_msgs::Transform* transform);

        /**
         * @brief Apply transform between two objects. The objects should contain a header but will only be transformed in translation space (and not orientation)
         * tf::Listener::transformPoint() can be used if the object T can be converted into a geometry_msgs::PointStamped and then converted back.
         * 
         * @tparam T 
         * @param from_frame 
         * @param target_frame 
         * @param object 
         * @param transformed_object 
         * @param time 
         */
        template<typename T>
        void apply_transform(const std::string& from_frame, const std::string& target_frame, const T& object, T* transformed_object, ros::Time& time = ros::Time());
    }

}


template<>
inline void VDO_SLAM::utils::apply_transform<nav_msgs::Odometry>(const std::string& from_frame, const std::string& target_frame, const nav_msgs::Odometry& object, nav_msgs::Odometry* transformed_object, ros::Time& time) {
    tf::TransformListener listener(ros::Duration(2));

    geometry_msgs::PointStamped object_point;
    object_point.header.frame_id = from_frame;

    object_point.header.stamp = time;

    //just an arbitrary point in space
    object_point.point.x = object.pose.pose.position.x;
    object_point.point.y = object.pose.pose.position.y;
    object_point.point.z = object.pose.pose.position.z;

    try {
        geometry_msgs::PointStamped transformed_point;

        listener.waitForTransform(target_frame, from_frame,
                              ros::Time::now(), ros::Duration(3.0));
        listener.transformPoint(target_frame, object_point, transformed_point);

        transformed_object->header.frame_id = target_frame;
        transformed_object->pose.pose.position.x = transformed_point.point.x;
        transformed_object->pose.pose.position.y = transformed_point.point.y;
        transformed_object->pose.pose.position.z = transformed_point.point.z;

    }
    catch(tf2::TransformException& ex) {
        ROS_ERROR_STREAM("Received an exception trying to transform a point from" << from_frame << " to " << target_frame <<": " <<  std::string(ex.what()));
    }
}

template<>
inline void VDO_SLAM::utils::apply_transform<cv::Point3d>(const std::string& from_frame, const std::string& target_frame, const cv::Point3d& object, cv::Point3d* transformed_object, ros::Time& time) {
    tf::TransformListener listener(ros::Duration(2));

    geometry_msgs::PointStamped object_point;

    object_point.header.frame_id = from_frame;
    //just an arbitrary point in space
    object_point.point.x = object.x;
    object_point.point.y = object.y;
    object_point.point.z = object.z;

    try {
        geometry_msgs::PointStamped transformed_point;

        listener.waitForTransform(target_frame, from_frame,
                              ros::Time::now(), ros::Duration(3.0));
        listener.transformPoint(target_frame, object_point, transformed_point);

        transformed_object->x = transformed_point.point.x;
        transformed_object->y = transformed_point.point.y;
        transformed_object->z = transformed_point.point.z;

    }
    catch(tf2::TransformException& ex) {
        ROS_ERROR_STREAM("Received an exception trying to transform a point from " << from_frame << " to " << target_frame << ": " <<  std::string(ex.what()));
    }
}

#endif
