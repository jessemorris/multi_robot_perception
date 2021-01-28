#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>



#include "RealTimeVdoSlam.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realtime_vdo_slam");
    ros::NodeHandle n;

    //lets just set up some static transforms for now
    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    //static transform between odom and map. We can update this later
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "odom";
    transform_stamped.transform.translation.x = 0;
    transform_stamped.transform.translation.y = 0;
    transform_stamped.transform.translation.z = 0;
    
    //must provide quaternion!
    transform_stamped.transform.rotation.x = 0;
    transform_stamped.transform.rotation.y = 0;
    transform_stamped.transform.rotation.z = 0;
    transform_stamped.transform.rotation.w = 1;

    static_broadcaster.sendTransform(transform_stamped);


    // std::string child_frame_id;
    // n.getParam("/frame_id", child_frame_id);
    // //make transform between odom and camera_link -> for now we say they are the same
    // transform_stamped.header.frame_id = "odom";
    // transform_stamped.child_frame_id = child_frame_id;
    // static_broadcaster.sendTransform(transform_stamped);

    ros::Rate r(2);

    RealTimeVdoSLAM real_time_vdom(n);

    // while(ros::ok()) {
    //     ros::spinOnce();
    //     r.sleep();
    // }
    ros::spin();
    
}