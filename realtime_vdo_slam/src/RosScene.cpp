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
#include <vdo_slam/Scene.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseWithCovariance.h>
#include <opencv2/core.hpp>

#include <mask_rcnn/MaskRcnnInterface.hpp>

using namespace mask_rcnn;


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
        //this one shouold have lavel
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

VDO_SLAM::RosScene::RosScene(Scene& _object, ros::Time _time, std::string& _frame_id, std::string& _child_frame_id) :
    time(_time),
    frame_id(_frame_id),
    child_frame_id(_child_frame_id),
    Scene(_object) {

        //convert them all into RosSceneObjects
        for(int i = 0; i < scene_objects.size(); i++) {
            RosSceneObject ros_scene_object(scene_objects[i], time);
            scene_objects[i] = ros_scene_object;
        }


        //update tf pose for ROS
        transform_stamped.header.stamp = time;
        transform_stamped.header.frame_id = frame_id;
        transform_stamped.child_frame_id = child_frame_id;
        transform_stamped.transform.translation.x = camera_pos_translation.x;
        transform_stamped.transform.translation.y = camera_pos_translation.y;
        transform_stamped.transform.translation.z = camera_pos_translation.z;

        tf2::Quaternion quat;
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
        odom.header.frame_id = frame_id;
        odom.child_frame_id = child_frame_id;
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
    time(_msg->header.stamp)
{   



}

const nav_msgs::Odometry& VDO_SLAM::RosScene::odom_msg() const {
    return odom;
}
const geometry_msgs::TransformStamped& VDO_SLAM::RosScene::tf_transform_msg() const {
    return transform_stamped;
}

const std::string& VDO_SLAM::RosScene::get_frame_id() {
    return frame_id;
}
const std::string& VDO_SLAM::RosScene::get_child_frame_id() {
    return child_frame_id;
}

const ros::Time& VDO_SLAM::RosScene::get_ros_time() {
    return time;
}

realtime_vdo_slam::VdoSlamScenePtr VDO_SLAM::RosScene::to_msg() {

}

void VDO_SLAM::RosScene::make_vizualisation(visualization_msgs::MarkerArray& marker_array) {
    for (SceneObject& scene_object: scene_objects) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = child_frame_id;
        marker.header.stamp = time;
        marker.ns = "vdoslam";
        marker.id = scene_object.tracking_id;
        // marker.type = visualization_msgs::Marker::SPHERE;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = scene_object.pose.x;
        marker.pose.position.y = scene_object.pose.y;
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
        // marker.text = scene_object.label;
        marker.text = scene_object.tracking_id;
        marker_array.markers.push_back(marker);
        // vis_count++;
    }

}
