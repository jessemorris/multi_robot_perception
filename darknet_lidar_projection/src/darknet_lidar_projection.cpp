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

#include "lidar_camera_projection/ImagePixelDepth.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include "darknet_lidar_projection/ClassifiedPoseArray.h"
#include "darknet_lidar_projection/ClassifiedPose.h"



#include <string>
#include <vector>
#include <algorithm>
#include <map>

//this currently only does one frame as it gets video from one stream

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class DarknetLidarFusion {

    public:
        DarknetLidarFusion(ros::NodeHandle& n) :
            handler(n),
            bounding_box_receievd(false),
            image_depth_received(false)
        {
            topic_prefix = "/darknet_lidar_projection/classified_poses";
            classified_pose_pub = handler.advertise<darknet_lidar_projection::ClassifiedPoseArray>("/darknet_lidar_projection/classified_poses", 10);
            vis_pub = handler.advertise<PointCloud>(topic_prefix + "/point_cloud", 10);
            marker_namespace = "classifed_pose";

            //get params
            handler.param<int>("bouding_box_epslon",bounding_box_epsilon,0);
        };

        void image_depth_call_back(const lidar_camera_projection::ImagePixelDepth::ConstPtr& msg);
        void bounding_box_call_back(const darknet_ros_msgs::BoundingBoxes::ConstPtr & msg);

        //only want to do this on lasest msg
        void fuse_data();

    private:
        lidar_camera_projection::ImagePixelDepth pixel_depth;
        darknet_ros_msgs::BoundingBoxes bounding_boxes;
        darknet_lidar_projection::ClassifiedPoseArray classified_pose_array;
        bool bounding_box_receievd;
        bool image_depth_received;
        ros::Publisher classified_pose_pub;

        //just for testing vis
        ros::Publisher vis_pub;
        std::map<std::string, ros::Publisher> classififer_pub_map_vis;
        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>> classififer_msgs_map_vis;

        //yolo classes to ppcl publishers for testing

        //yolo classes to publishers
        std::map<std::string, ros::Publisher> classififer_pub_map;
        std::map<std::string, darknet_lidar_projection::ClassifiedPoseArray> classififer_msgs_map;
        std::string topic_prefix;
        ros::NodeHandle handler;
        std::string marker_namespace;

        //params
        int bounding_box_epsilon;

};


void DarknetLidarFusion::image_depth_call_back(const lidar_camera_projection::ImagePixelDepth::ConstPtr& msg) {
    pixel_depth = *msg;
    image_depth_received = true;
    //ROS_INFO("depth message received at time");
}



void DarknetLidarFusion::bounding_box_call_back(const darknet_ros_msgs::BoundingBoxes::ConstPtr & msg) {
    bounding_boxes = *msg;
    bounding_box_receievd = true;
    //ROS_INFO("bounding box message received at time");


}

 void DarknetLidarFusion::fuse_data() {

    classified_pose_array.data.clear();
    std::map<std::string, darknet_lidar_projection::ClassifiedPoseArray>::iterator pose_it;
    for(pose_it = classififer_msgs_map.begin(); pose_it != classififer_msgs_map.end(); pose_it++) {
        pose_it->second.data.clear();
    }

    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>>::iterator pose_it_vis;
    for(pose_it_vis = classififer_msgs_map_vis.begin(); pose_it_vis != classififer_msgs_map_vis.end(); pose_it_vis++) {
        pose_it_vis->second.points.clear();
    }

    if (!image_depth_received || !bounding_box_receievd) {
        return;
    }
    //marker_counter = 0;

    //ROS_INFO("fusing data");
    //bounding_box_receievd = false;
    //these should be copies so we dont overwrite our old boxes
    lidar_camera_projection::ImagePixelDepth current_pixel_depth = pixel_depth;
    darknet_ros_msgs::BoundingBoxes current_bounding_boxes = bounding_boxes;

    //sort arrays by x and then y
    std::vector<lidar_camera_projection::PixelDepth> pixel_depth_array(current_pixel_depth.data);

    //ROS_INFO_STREAM("length " << pixel_depth_array.size() );
    PointCloud pcl_msg;

    //go through all the bounding boxes because probably less of them
    //really should be doing range tree here but for now
    std::vector<darknet_ros_msgs::BoundingBox> bounding_box_list(current_bounding_boxes.bounding_boxes);
    for(const darknet_ros_msgs::BoundingBox& box : bounding_box_list) {
        //ROS_INFO_STREAM("bounding box x " << box.xmin << " y " << box.ymin);

        for(const lidar_camera_projection::PixelDepth& pixel : pixel_depth_array) {
            //if is whthin bounding box
            if (pixel.pixel_x < (box.xmax - bounding_box_epsilon) && pixel.pixel_x > (box.xmin + bounding_box_epsilon) 
                    && pixel.pixel_y < (box.ymax - bounding_box_epsilon) && pixel.pixel_y > (box.ymin + bounding_box_epsilon)) {

                //is a new class
                if (classififer_pub_map.find(box.Class) == classififer_pub_map.end()) {

                    //do some handling in case the classes have whitespaces in them
                    std::string class_name = box.Class;
                    std::size_t pos = class_name.find(" ");

                    if (pos != std::string::npos) {
                        class_name = class_name.replace(pos, std::string(" ").length(), "_");
                    }

                    classififer_pub_map[box.Class] = handler.advertise<darknet_lidar_projection::ClassifiedPoseArray>(topic_prefix + "/" + class_name, 10);
                    classififer_pub_map_vis[box.Class] = handler.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic_prefix + "/points/" + class_name, 10);
                }

                darknet_lidar_projection::ClassifiedPose classified_pose;
                classified_pose.Class = box.Class;
                classified_pose.probability = box.probability;
                classified_pose.x = pixel.x;
                classified_pose.y = pixel.y;
                classified_pose.z = pixel.z;


                pcl_msg.header.frame_id = current_pixel_depth.header.frame_id;
                // ROS_INFO_STREAM("frame info " << current_pixel_depth.header.frame_id);
                pcl_msg.height = pcl_msg.width = 0.02;
                pcl::PointXYZ point(classified_pose.x, classified_pose.y, classified_pose.z);


                //ROS_INFO_STREAM("added point x " << classified_pose.x << " y " << classified_pose.y << " z " << classified_pose.z);
                pcl_msg.points.push_back(point);

                classified_pose_array.data.push_back(classified_pose);
                classififer_msgs_map[box.Class].data.push_back(classified_pose);

                classififer_msgs_map_vis[box.Class].points.push_back(point);
                classififer_msgs_map_vis[box.Class].header.frame_id = current_pixel_depth.header.frame_id;
                classififer_msgs_map_vis[box.Class].height = classififer_msgs_map_vis[box.Class].width = 0.02;
                
            }
        }
    }

    classified_pose_pub.publish(classified_pose_array);

    //testing
    pcl_conversions::toPCL(ros::Time::now(), pcl_msg.header.stamp);
    vis_pub.publish (pcl_msg);
    //ROS_INFO_STREAM("published");


    //also publish all sub topics
    std::map<std::string, ros::Publisher>::iterator it;
    for(it = classififer_pub_map.begin(); it != classififer_pub_map.end(); it++) {
        std::string key = it->first;
        it->second.publish(classififer_msgs_map[key]);
    }

    for(it = classififer_pub_map_vis.begin(); it != classififer_pub_map_vis.end(); it++) {
        std::string key = it->first;
        pcl_conversions::toPCL(ros::Time::now(), classififer_msgs_map_vis[key].header.stamp);
        it->second.publish(classififer_msgs_map_vis[key]);
    }




 }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "darknet_lidar_projection");
    ros::NodeHandle n;

    ros::Rate r(10);

    std::string image_depth_topic, bounding_box_topic;
    n.getParam("darknet_lidar_projection/image_depth_topic", image_depth_topic);
    n.getParam("darknet_lidar_projection/darknet_bounding_box_topic", bounding_box_topic);

    DarknetLidarFusion darknet_lidar_fusion(n);

    ros::Subscriber image_depth_sub = n.subscribe<lidar_camera_projection::ImagePixelDepth>(image_depth_topic, 10, &DarknetLidarFusion::image_depth_call_back, &darknet_lidar_fusion);
    ros::Subscriber bounding_box_sub = n.subscribe<darknet_ros_msgs::BoundingBoxes>(bounding_box_topic, 10, &DarknetLidarFusion::bounding_box_call_back, &darknet_lidar_fusion);



    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub_img = it.advertise(output_video_topic, 1);
    while(ros::ok()) {
        darknet_lidar_fusion.fuse_data();
        ros::spinOnce();
        r.sleep();
    }
    
}