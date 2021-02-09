#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>



#include "RealTimeVdoSlam.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>

float odom_x_offset = -127.7;
float odom_y_offset = -90.88;

nav_msgs::Odometry odom_new;

void odom_repub_callback(const nav_msgs::OdometryConstPtr& msg) {
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    header.frame_id = "odom_offset";
    odom_new.header = header;
    odom_new.pose.pose.position.x = msg->pose.pose.position.x - odom_x_offset;
    odom_new.pose.pose.position.y = msg->pose.pose.position.y - odom_y_offset;
    odom_new.pose.pose.position.z = 0;

    odom_new.pose.pose.orientation = msg->pose.pose.orientation;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_vdoslam");
    ros::NodeHandle n;

    //lets just set up some static transforms for now
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener(tf_buffer);
    tf2_ros::StaticTransformBroadcaster static_broadcaster;


    ros::Publisher odom_repub = n.advertise<nav_msgs::Odometry>("/odom_repub", 1000);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_repub_callback);

    //lets listen to the current info from the bag file and set vdo_odom to be relative to the first
    //transform between the gt map and gt odom (so they start at the same point)
    // geometry_msgs::TransformStamped transform;
    // try {
    //     //wait for transform doesnt exist in tf2_ros so we wait for a long time?
    //     transform = tf_buffer.lookupTransform("map", "odom",  
    //     ros::Time::now(), ros::Duration(1000));
    // }
    // catch (tf2::TransformException& ex) {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    //make new vdo_map frame that will be new starting reference point refernce point 
    //we are trying to put /odom and /vdo_odom at the same location for plotting
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time(1);
    transform_stamped.header.frame_id = "vdo_map";
    transform_stamped.child_frame_id = "vdo_odom";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;
    
    //must provide quaternion!
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;
    static_broadcaster.sendTransform(transform_stamped);
    ros::spinOnce();

    transform_stamped.child_frame_id = "odom_offset";
    static_broadcaster.sendTransform(transform_stamped);
    ros::spinOnce();

    ros::Rate r(2);
    RosVdoSlam ros_vdo_slam(n);

    // ROS_INFO_STREAM("Waiting for /odom for initalisation");
    // nav_msgs::OdometryConstPtr odom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");


    // //static transform between odom and map. We can update this late
    // transform_stamped.header.stamp = odom->header.stamp;
    // transform_stamped.header.frame_id = "map";
    // transform_stamped.child_frame_id = "vdo_odom";
    // transform_stamped.transform.translation.x = -odom->pose.pose.position.x;
    // transform_stamped.transform.translation.y = -odom->pose.pose.position.y;
    // transform_stamped.transform.translation.z = 0;
    
    // //must provide quaternion!
    // transform_stamped.transform.rotation.x = odom->pose.pose.orientation.x;
    // transform_stamped.transform.rotation.y = odom->pose.pose.orientation.y;
    // transform_stamped.transform.rotation.z = odom->pose.pose.orientation.z;
    // transform_stamped.transform.rotation.w = odom->pose.pose.orientation.w;

    // static_broadcaster.sendTransform(transform_stamped);
    // // ros::spinOnce();

    // transform_stamped.header.stamp = ros::Time::now();
    // transform_stamped.header.frame_id = "vdo_map";
    // transform_stamped.child_frame_id = "odom";
    // transform_stamped.transform.translation.x = odom->pose.pose.position.x;
    // transform_stamped.transform.translation.y = odom->pose.pose.position.y;
    // transform_stamped.transform.translation.z = 0;

    // //must provide quaternion!
    // transform_stamped.transform.rotation.x = 0;
    // transform_stamped.transform.rotation.y = 0;
    // transform_stamped.transform.rotation.z = 0;
    // transform_stamped.transform.rotation.w = 1;
    // static_broadcaster.sendTransform(transform_stamped);
    // ros::spinOnce();

    while(ros::ok()) {
        ros::spinOnce();
        odom_repub.publish(odom_new);
        r.sleep();
    }
    // ros::spin();
    
}