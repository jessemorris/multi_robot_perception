#include "visualizer/RosVisualizer.hpp"

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>

#include "utils/RosUtils.hpp"


using namespace VDO_SLAM;


cv::Mat VDO_SLAM::overlay_scene_image(const cv::Mat& image, const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
    cv::Mat overlayed;
    image.copyTo(overlayed);


    std::vector<realtime_vdo_slam::VdoSceneObject> objects = slam_scene->objects;

    for(realtime_vdo_slam::VdoSceneObject& object : objects) {
        //draw bounding box
        cv::rectangle(overlayed, cv::Rect2d(object.bounding_box.center.x, object.bounding_box.center.y,
            object.bounding_box.size_x, object.bounding_box.size_y), 
            cv::Scalar(0, 255, 0), 2);


        //add info
        char text[200];
        sprintf(text, "%s, [%d] %02fkm/h", object.label.c_str(), object.tracking_id, object.twist.linear.x);
        cv::putText(overlayed, text, cv::Point(object.bounding_box.center.x, object.bounding_box.center.y), 
            cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

    }

    return overlayed;
}

namespace VDO_SLAM {


    RosVisuazlier::RosVisuazlier(ros::NodeHandle& _nh)
        : nh(_nh),
          listener(tf_buffer) {
              slam_scene_pub = nh.advertise<realtime_vdo_slam::VdoSlamScene>("vdoslam/output/scene", 20);
              slam_scene_3d_pub = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/output/3dscene", 20);

              nh.getParam("/ros_vdoslam/odometry_ground_truth_topic", odom_gt_topic);

            if(!odom_gt_topic.empty()) {
                odom_gt_sub = nh.subscribe<nav_msgs::Odometry>(odom_gt_topic, 100, &RosVisuazlier::odom_gt_callback, this);
            }

                //Getting Frame ID's
            nh.getParam("/ros_vdoslam/map_frame_id", map_frame);
            nh.getParam("/ros_vdoslam/odom_frame_id", odom_frame);
            nh.getParam("/ros_vdoslam/base_link_frame_id", base_frame);


            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = 0;
            odom.pose.pose.position.y = 0;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;

            //setting map frame and odom frame to be the same
            VDO_SLAM::utils::publish_static_tf(odom, map_frame, odom_frame);

            //setting base link starting point to be the same as odom
            VDO_SLAM::utils::publish_static_tf(odom, odom_frame, base_frame);


        }

}