#include "RosScene.hpp"

#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>



int VDO_SLAM::RosSceneManager::vis_count = 0;

//should also convert unix timestamp to ROS time
//current timetstamp is just time difference and not unix time
VDO_SLAM::RosSceneObject::RosSceneObject(SceneObject& _object, ros::Time& _time, int _uid) :
    SceneObject(_object),
    time(_time),
    uid(_uid) {}



VDO_SLAM::RosSceneManager::RosSceneManager(ros::NodeHandle& _nh) :
        nh(_nh)
    {
        visualiser = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/visualization", 20 );
        odom_pub = nh.advertise<nav_msgs::Odometry>("vdoslam/odom", 20);
        nh.getParam("/frame_id", child_frame_id);
    }

void VDO_SLAM::RosSceneManager::display_scene(RosScenePtr& scene) {
    const nav_msgs::Odometry odom = scene->odom_msg();
    const geometry_msgs::TransformStamped tf = scene->tf_transform_msg();

    visualization_msgs::MarkerArray marker_array;
    scene->make_vizualisation(marker_array);

    odom_pub.publish(odom);
    visualiser.publish(marker_array);
    broadcaster.sendTransform(tf);

}

void VDO_SLAM::RosSceneManager::to_cv_mat(cv::Mat& display) {

}

VDO_SLAM::RosScene::RosScene(Scene& _object, ros::Time _time) :
    time(_time),
    Scene(_object) {

        ROS_INFO_STREAM("Camera pos: " << camera_pos_translation);
        ROS_INFO_STREAM("Camera rot: " << camera_pos_rotation);

        //convert them all into RosSceneObjects
        int id = 0;
        for(int i = 0; i < scene_objects.size(); i++) {
            RosSceneObject ros_scene_object(scene_objects[i], time, id);
            scene_objects[i] = ros_scene_object;
            id++;
        }


        //update tf pose for ROS
        transform_stamped.header.stamp = time;
        transform_stamped.header.frame_id = "vdo_odom";
        //TODO: want this to be param eventaully but lazy design currently and lack of time to refactor
        transform_stamped.child_frame_id = "vdo_camera_link";
        transform_stamped.transform.translation.x = -camera_pos_translation.x;
        transform_stamped.transform.translation.y = -camera_pos_translation.y;
        transform_stamped.transform.translation.z = 0;

        tf2::Quaternion quat;
        ROS_INFO_STREAM("making transform");
        tf2::Matrix3x3 rotation_matrix_pos(camera_pos_rotation.at<float>(0, 0), camera_pos_rotation.at<float>(0, 1), camera_pos_rotation.at<float>(0, 2),
                                           camera_pos_rotation.at<float>(1, 0), camera_pos_rotation.at<float>(1, 1), camera_pos_rotation.at<float>(1, 2),
                                           camera_pos_rotation.at<float>(2, 0), camera_pos_rotation.at<float>(2, 1), camera_pos_rotation.at<float>(2, 2));
                

        rotation_matrix_pos.getRotation(quat);
        
        //must provide quaternion!
        transform_stamped.transform.rotation.x = quat.x();
        transform_stamped.transform.rotation.y = quat.y();
        transform_stamped.transform.rotation.z = quat.z();
        transform_stamped.transform.rotation.w = quat.w();

        // ROS_INFO_STREAM("Published transform");
        odom.header.stamp = time;
        odom.header.frame_id = "vdo_odom";
        odom.child_frame_id = "vdo_camera_link";
        odom.pose.pose.position.x = -camera_pos_translation.x;
        odom.pose.pose.position.y = -camera_pos_translation.y;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        odom.twist.twist.linear.x = camera_vel_translation.x;
        odom.twist.twist.linear.y = camera_vel_translation.y;
        odom.twist.twist.linear.z = camera_vel_translation.z;

        //TODO: angular velocity

    }
const nav_msgs::Odometry& VDO_SLAM::RosScene::odom_msg() const {
    return odom;
}
const geometry_msgs::TransformStamped& VDO_SLAM::RosScene::tf_transform_msg() const {
    return transform_stamped;
}

void VDO_SLAM::RosScene::make_vizualisation(visualization_msgs::MarkerArray& marker_array) {
    for (SceneObject& scene_object: scene_objects) {
        ROS_INFO_STREAM(scene_object);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "vdo_odom";
        marker.header.stamp = time;
        marker.ns = "vdoslam";
        marker.id = scene_object.tracking_id;
        // marker.type = visualization_msgs::Marker::SPHERE;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = -scene_object.pose.x;
        marker.pose.position.y = -scene_object.pose.y;
        marker.pose.position.z = 0;



        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        marker.text = scene_object.label;

        marker_array.markers.push_back(marker);
        // vis_count++;
    }

}
