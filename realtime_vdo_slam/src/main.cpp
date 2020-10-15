#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "RealTimeVdoSlam.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "realtime_vdo_slam");
    ros::NodeHandle n;

    ros::Rate r(10);

    RealTimeVdoSLAM real_time_vdom(n);

    while(ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    
}