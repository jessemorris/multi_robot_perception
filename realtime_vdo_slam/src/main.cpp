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

    std::string image_topic;
    n.getParam("darknet_lidar_projection/image_topic", image_topic);

    RealTimeVdoSLAM real_time_vdom(n);

    // ros::Subscriber image_depth_sub = n.subscribe<lidar_camera_projection::ImagePixelDepth>(image_depth_topic, 10, &DarknetLidarFusion::image_depth_call_back, &darknet_lidar_fusion);
    // ros::Subscriber bounding_box_sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>(bounding_box_topic, 10, &DarknetLidarFusion::bounding_box_call_back, &darknet_lidar_fusion);



    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub_img = it.advertise(output_video_topic, 1);
    ros::spin();
    // while(ros::ok()) {
    //     darknet_lidar_fusion.fuse_data();
    //     ros::spinOnce();
    //     r.sleep();
    // }
    
}