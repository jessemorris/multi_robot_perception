#include "RosScene.hpp"

#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <vdo_slam/Scene.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <opencv2/core.hpp>
;


/**
 * @brief Converts a segmentation index (ie. a pixel level segmentation or the classification index
 * assignment to a scene object) to a colour so each different classification can be visualisaed.
 * 
 * Uses the MaskRcnnInterface to get all the available categories and divides the RGB colour specitrum amongst
 * them evenly.
 * 
 * @param index 
 * @return cv::Scalar 
 */
cv::Scalar seg_index_to_colour(int index) {
    ROS_INFO_STREAM(index);
    // int total_categories = MaskRcnnInterface::categories_size();

    //as a hack assume 20 categories so we can get nice clear colours
    int total_categories = 20;
    ROS_INFO_STREAM(total_categories);
    int options = (255.0 * 3.0/(float)total_categories);

    int r = 0;
    int g = 0;
    int b = 0;

    int result = index * options;
    ROS_INFO_STREAM("Result " << result);

    if (result > 510) {
        r = 255;
        g = 255;
        result -= 255;

    }

    if (result > 255) {
        g = 255;
        
        result -= 255;
        b = result;
    }

    b = result;

    return cv::Scalar(r, g, b);


}


//should also convert unix timestamp to ROS time
//current timetstamp is just time difference and not unix time

VDO_SLAM::RosSceneObject::RosSceneObject(realtime_vdo_slam::VdoSceneObjectConstPtr& _msg) : 
    time(_msg->time) {

        //TODO: currently ignoring rotation
        pose.x = _msg->pose.position.x;
        pose.y = _msg->pose.position.y;
        pose.z = _msg->pose.position.z;

        velocity.x = _msg->twist.linear.x;
        velocity.y = _msg->twist.linear.y;

        semantic_instance_index = _msg->semantic_label;
        //this one should have lavel
        //TODO: some assert if label not default - shoudl set this somehwere?
        label = _msg->label;
        tracking_id = _msg->tracking_id;

    }

VDO_SLAM::RosSceneObject::RosSceneObject(SceneObject& _object, ros::Time& _time) :
    SceneObject(_object),
    time(_time) {}

realtime_vdo_slam::VdoSceneObjectPtr VDO_SLAM::RosSceneObject::to_msg() {
    realtime_vdo_slam::VdoSceneObjectPtr msg(new realtime_vdo_slam::VdoSceneObject);
    msg->pose.position.x = pose.x;
    msg->pose.position.y = pose.y;
    msg->pose.position.z = pose.z;

    msg->twist.linear.x = velocity.x;
    msg->twist.linear.y = velocity.y;
    msg->semantic_label = semantic_instance_index;

    //we should not label here becuase the scene object may not have the correct label
    msg->tracking_id = tracking_id;
    msg->time = time;

    return msg;

}

VDO_SLAM::RosScene::RosScene(Scene& _object, ros::Time _time) :
    time(_time),
    Scene(_object) {

        //convert them all into RosSceneObjects
        for(int i = 0; i < scene_objects.size(); i++) {
            RosSceneObject ros_scene_object(scene_objects[i], time);
            scene_objects[i] = ros_scene_object;
        }

        tf2::Quaternion quat;
        tf2::Matrix3x3 rotation_matrix_pos(camera_pos_rotation.at<float>(0, 0), camera_pos_rotation.at<float>(0, 1), camera_pos_rotation.at<float>(0, 2),
                                           camera_pos_rotation.at<float>(1, 0), camera_pos_rotation.at<float>(1, 1), camera_pos_rotation.at<float>(1, 2),
                                           camera_pos_rotation.at<float>(2, 0), camera_pos_rotation.at<float>(2, 1), camera_pos_rotation.at<float>(2, 2));
                

        rotation_matrix_pos.getRotation(quat);
        
        //Note: we dont add transform frame and child frame here
        odom.header.stamp = time;
        odom.pose.pose.position.x = camera_pos_translation.x;
        odom.pose.pose.position.y = camera_pos_translation.y;
        odom.pose.pose.position.z = camera_pos_translation.z;

        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        odom.twist.twist.linear.x = camera_vel_translation.x;
        odom.twist.twist.linear.y = camera_vel_translation.y;
        odom.twist.twist.linear.z = camera_vel_translation.z;

        //TODO: angular velocity

}


VDO_SLAM::RosScene::RosScene(realtime_vdo_slam::VdoSlamSceneConstPtr& _msg) :
    time(_msg->header.stamp) {}

const nav_msgs::Odometry& VDO_SLAM::RosScene::odom_msg() const {
    return odom;
}


const ros::Time& VDO_SLAM::RosScene::get_ros_time() {
    return time;
}

realtime_vdo_slam::VdoSlamScenePtr VDO_SLAM::RosScene::to_msg() {

}
