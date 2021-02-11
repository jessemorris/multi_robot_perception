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

//for a1_video_full
// float odom_x_offset = -127.7;
// float odom_y_offset = -90.88;

//for al_video_begin


class VdoUtils {

    public:
        VdoUtils(ros::NodeHandle& _nh) :
                nh(_nh) ,
                listener(tf_buffer),
                is_first(true)
            {
                odom_x_offset = 0;
                odom_y_offset = 0;
                odom_repub = nh.advertise<nav_msgs::Odometry>("/odom_repub", 1000);
                odom_sub = nh.subscribe("/odom", 1000, &VdoUtils::odom_repub_callback, this);

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

            }

        void odom_repub_callback(const nav_msgs::OdometryConstPtr& msg) {
            if (is_first) {
                odom_x_offset = msg->pose.pose.position.x;
                odom_y_offset = msg->pose.pose.position.y;
                is_first = false;
            }
            std_msgs::Header header = std_msgs::Header();
            header.stamp = ros::Time::now();
            header.frame_id = "odom_offset";
            odom_new.header = header;
            odom_new.pose.pose.position.x = msg->pose.pose.position.x - odom_x_offset;
            odom_new.pose.pose.position.y = msg->pose.pose.position.y - odom_y_offset;
            odom_new.pose.pose.position.z = 0;

            odom_new.pose.pose.orientation = msg->pose.pose.orientation;

            odom_repub.publish(odom_new);
        }


    private:
        ros::NodeHandle nh;
        nav_msgs::Odometry odom_new;
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener listener;
        tf2_ros::StaticTransformBroadcaster static_broadcaster;

        ros::Publisher odom_repub;
        ros::Subscriber odom_sub;
        bool is_first;
        float odom_x_offset;
        float odom_y_offset;

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_vdoslam");
    ros::NodeHandle n;

    VdoUtils utils(n);
    RosVdoSlam ros_vdo_slam(n);
    ros::spin();
    
}