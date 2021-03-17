#include "visualizer/RosVisualizer.hpp"

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>
#include <nav_msgs/Odometry.h>

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


    RosVisualizer::RosVisualizer(ros::NodeHandle& _nh)
        : nh(_nh),
          image_transport(nh),
          listener(tf_buffer) {

            slam_scene_pub = nh.advertise<realtime_vdo_slam::VdoSlamScene>("vdoslam/output/scene", 20);
            slam_scene_3d_pub = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/output/3dscene", 20);

            bounding_box_pub = image_transport.advertise("vdoslam/output/bounding_box_image", 20);
            object_track_pub = image_transport.advertise("vdoslam/output/object_point_image", 20);

            nh.getParam("/ros_vdoslam/odometry_ground_truth_topic", odom_gt_topic);

            if(!odom_gt_topic.empty()) {
                odom_gt_sub = nh.subscribe<nav_msgs::Odometry>(odom_gt_topic, 100, &RosVisualizer::odom_gt_callback, this);
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

            ROS_INFO_STREAM("Created VDO_SLAM visualizer");

    }

    RosVisualizer::~RosVisualizer() {
        slam_scene_queue.shutdown();
    }

    bool RosVisualizer::spin_viz(int rate) {
        realtime_vdo_slam::VdoSlamScenePtr slam_scene;
        ROS_INFO_STREAM("Starting viz with rate: " << rate << "Hz.");

        ros::Rate r(rate);
        size_t rate_ms = rate * 1000; //convert to ms
        while(ros::ok() && !slam_scene_queue.isShutdown()) {
            if(slam_scene_queue.popBlockingWithTimeout(slam_scene, rate_ms)) {
                ROS_INFO_STREAM("Spinning");
                update_spin(slam_scene);
            }
            r.sleep();    
        }
    }

    bool RosVisualizer::queue_slam_scene(realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
        ROS_INFO_STREAM("added");
        return slam_scene_queue.push(slam_scene);
    }

    void RosVisualizer::odom_gt_callback(const nav_msgs::OdometryConstPtr& msg) {
        if (is_first) {
            odom_x_offset = msg->pose.pose.position.x;
            odom_y_offset = msg->pose.pose.position.y;
            is_first = false;
        }
        gt_odom.pose.pose.position.x = odom_x_offset - msg->pose.pose.position.x ;
        gt_odom.pose.pose.position.y = odom_y_offset - msg->pose.pose.position.y;
        gt_odom.pose.pose.position.z = 0;

        gt_odom.pose.pose.orientation = msg->pose.pose.orientation;
    }

    bool RosVisualizer::update_spin(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
        publish_odom(slam_scene);
        publish_3D_viz(slam_scene);
        publish_3D_viz(slam_scene);
        publish_display_mat(slam_scene);
        publish_bounding_box_mat(slam_scene);
    }

    void RosVisualizer::publish_odom(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
        nav_msgs::Odometry odom;
        utils::geometry_to_odom(slam_scene->camera_pose, slam_scene->camera_twist, 
            slam_scene->header.stamp, odom_frame, base_frame, odom);

        odom_pub.publish(odom);

        utils::publish_static_tf(odom, odom_frame, base_frame);

    }

    void RosVisualizer::publish_3D_viz(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {

    }

    void RosVisualizer::publish_display_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene) {

    }

    void RosVisualizer::publish_bounding_box_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene) {
        cv::Mat raw_img;
        utils::image_msg_to_mat(raw_img, scene->original_frame, sensor_msgs::image_encodings::RGB8);
        cv::Mat viz = overlay_scene_image(raw_img, scene);

        sensor_msgs::ImagePtr img_ptr;

        utils::image_to_msg_ptr(viz, img_ptr, sensor_msgs::image_encodings::RGB8, scene->header);

        bounding_box_pub.publish(*img_ptr);

    }







}