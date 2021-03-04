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

int VDO_SLAM::RosSceneManager::vis_count = 0;

//should also convert unix timestamp to ROS time
//current timetstamp is just time difference and not unix time

VDO_SLAM::RosSceneObject::RosSceneObject(realtime_vdo_slam::VdoSceneObjectConstPtr& _msg) : 
    time(_msg->time),
    uid(_msg->uid) {

        //TODO: currently ignoring rotation
        pose.x = _msg->pose.position.x;
        pose.y = _msg->pose.position.y;
        pose.z = _msg->pose.position.z;

        velocity.x = _msg->twist.linear.x;
        velocity.y = _msg->twist.linear.y;

        label_index = _msg->semantic_label;
        label = _msg->label;
        tracking_id = _msg->tracking_id;

    }

VDO_SLAM::RosSceneObject::RosSceneObject(SceneObject& _object, ros::Time& _time, int _uid) :
    SceneObject(_object),
    time(_time),
    uid(_uid) {}

realtime_vdo_slam::VdoSceneObjectPtr VDO_SLAM::RosSceneObject::to_msg() {
    realtime_vdo_slam::VdoSceneObjectPtr msg(new realtime_vdo_slam::VdoSceneObject);
    msg->pose.position.x = pose.x;
    msg->pose.position.y = pose.y;
    msg->pose.position.z = pose.z;

    msg->twist.linear.x = velocity.x;
    msg->twist.linear.y = velocity.y;

    msg->semantic_label = label_index;
    msg->label = label;
    msg->tracking_id = tracking_id;

    msg->uid = uid;
    msg->time = time;

    return msg;

}

VDO_SLAM::RosSceneManager::RosSceneManager(ros::NodeHandle& _nh) :
        nh(_nh),
        listener(tf_buffer)
    {
        visualiser = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/visualization", 20 );
        odom_pub = nh.advertise<nav_msgs::Odometry>("vdoslam/odom", 20);
        odom_repub_sub = nh.subscribe<nav_msgs::Odometry>("/odom_repub", 100, &RosSceneManager::odom_repub_callback, this);
        nh.getParam("/frame_id", child_frame_id);

        display = cv::Mat::zeros(800, 800, CV_8UC3);
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

void VDO_SLAM::RosSceneManager::odom_repub_callback(const nav_msgs::OdometryConstPtr& msg) {
    double x = -msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    //here we update the odom repub to the display mat
    //we use 10 for scale
    
    int x_display = (x * scale) + x_offset;
    int y_display = (y * scale) + y_offset;
    //add odom to cv mat
    display_mutex.lock();
    cv::rectangle(display, cv::Point(y_display, x_display), cv::Point(y_display+10, x_display+10), cv::Scalar(0,255,0),1);
    // cv::rectangle(display, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), CV_FILLED);
    // cv::putText(display, "Camera GT Trajectory (GREEN SQUARE)", cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    // char text[100];
    // sprintf(text, "x = %02fm y = %02fm z = %02fm", x, y, z);
    // cv::putText(display, text, cv::Point(10, 120), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);
    display_mutex.unlock();
}

cv::Mat& VDO_SLAM::RosSceneManager::get_display_mat() {
    return display;
}

void VDO_SLAM::RosSceneManager::update_display_mat(std::unique_ptr<VDO_SLAM::RosScene>& scene) {
    const nav_msgs::Odometry odom = scene->odom_msg();

    //camera is offset from the base link so try and orietnate 
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::Pose pose;
    pose.position = odom.pose.pose.position;
    pose.orientation = odom.pose.pose.orientation;

    geometry_msgs::Pose transformed_pose;
    
    // try {
	// 	transform_stamped = tf_buffer.lookupTransform("base_link", "vdo_camera_link", ros::Time(0));
	// } catch (const tf2::TransformException& e) {
	// 	ROS_WARN_STREAM("Skipping track_callback because lookup_transform failed with exception: " << e.what());
	// 	return;
	// }

    tf2::doTransform(pose, transformed_pose, transform_stamped);



    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;
    // double x = transformed_pose.position.x;
    // double y = transformed_pose.position.y;
    // double z = transformed_pose.position.z;


    

    //800 is height of cv mat and we want to draw from bottom left
    int x_display =  static_cast<int>(x*scale) + x_offset;
    int y_display =  static_cast<int>(y*scale) + y_offset;
    ROS_INFO_STREAM("x display " << x_display << " y display " << y_display);
    //add odom to cv mat
    display_mutex.lock();
    cv::rectangle(display, cv::Point(x_display, y_display), cv::Point(x_display+10, y_display+10), cv::Scalar(0,0,255),1);
    cv::rectangle(display, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(display, "Camera Trajectory (RED SQUARE)", cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    char text[100];
    sprintf(text, "x = %02fm y = %02fm z = %02fm", x, y, z);
    cv::putText(display, text, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);
    display_mutex.unlock();


    //draw each object
    std::vector<SceneObject> scene_objects = scene->get_scene_objects();

    for(SceneObject& scene_object : scene_objects) {
        int x = scene_object.pose.x;
        int y = scene_object.pose.y;

        int x_display =  static_cast<int>(x*scale) + x_offset;
        int y_display =  static_cast<int>(y*scale) + y_offset;

        // cv::Scalar colour = seg_index_to_colour(scene_object.label_index);
        // cv::circle(display, cv::Point(x_display, y_display), 2, colour, 5); // orange

        // TODO: use colour conversion function
        switch (scene_object.label_index) {

            case 1:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // orange
                break;
            case 2:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,125,125), 5); // green
                break;
            case 3:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0, 255, 255), 5); // yellow
                break;
            case 4:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // pink
                break;
            case 5:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,0), 5); // cyan (yellow green 47,255,173)
                break;
            case 6:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // purple
                break;
            case 7:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,255), 5);  // white
                break;
            case 8:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(196,228,255), 5); // bisque
                break;
            case 9:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(180, 105, 255), 5);  // blue
                break;
            case 10:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(42,42,165), 5);  // brown
                break;
            default:
                ROS_WARN_STREAM("No case for this segmentaion index yet");
                break;
        }

    }


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
        odom.header.frame_id = "vdo_odom";
        odom.child_frame_id = "vdo_camera_link";
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
    time(_msg->header.stamp),
    Scene(_msg->id)
{   



}

const nav_msgs::Odometry& VDO_SLAM::RosScene::odom_msg() const {
    return odom;
}
const geometry_msgs::TransformStamped& VDO_SLAM::RosScene::tf_transform_msg() const {
    return transform_stamped;
}

realtime_vdo_slam::VdoSlamScenePtr VDO_SLAM::RosScene::to_msg() {

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
        marker.text = scene_object.label;

        marker_array.markers.push_back(marker);
        // vis_count++;
    }

}
