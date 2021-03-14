#include "ImagePreprocessingRGB.hpp"


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>



// #incl
#include <mono_depth_2/MonoDepthInterface.hpp>
#include <mask_rcnn/MaskRcnnInterface.hpp>
#include <mask_rcnn/SemanticTracker.hpp>
#include <flow_net/FlowNetInterface.hpp>

using namespace VDO_SLAM::preprocessing;

ImageRGB::ImageRGB(ros::NodeHandle& n):
    BaseProcessing(n) {
        image_subscriber = image_transport.subscribe(input_camera_topic, 20,
                                               &ImageRGB::image_callback, this);
    }

void ImageRGB::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat distored = cv_ptr->image;
    cv::Mat undistorted;
    // distored.copyTo(undistorted);
    
    cv::Mat image;
    if (undistord_images) {
        undistortImage(distored, undistorted);
        image = undistorted;
    }
    else {
        image = distored;
    }

    std_msgs::Header original_header = msg->header;

    std::vector<std::string> mask_rcnn_labels;
    std::vector<int> mask_rcnn_label_indexs;
    std::vector<vision_msgs::BoundingBox2D> bb;
    std::vector<mask_rcnn::SemanticObject> semantic_objects;

    cv::Mat tracked_mask;
    cv::Mat tracked_viz;


    if (is_first) {
        previous_image = image;
        previous_time = msg->header.stamp;
        is_first = false;
        return;
    }
    else {
        cv::Mat current_image = image;
        current_time = msg->header.stamp;

        // //TODO: what should this be
        // original_header.frame_id = "base_link";
        if (run_scene_flow) {
            scene_flow_success = sceneflow.analyse(current_image, previous_image, scene_flow_mat, scene_flow_viz);

            if (scene_flow_success) {
                //#TODO: cannot fisplay until convert from scene flow to rgb               
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "32FC2", scene_flow_mat).toImageMsg();
                flownet_raw.publish(img_msg);

                img_msg = cv_bridge::CvImage(original_header, "rgb8", scene_flow_viz).toImageMsg();
                flownet_viz.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse scene flow images");
            }
        }

        if (run_mask_rcnn) {
            mask_rcnn_success = mask_rcnn_interface.analyse(current_image, mask_rcnn_mat, mask_rcnn_viz, mask_rcnn_labels, mask_rcnn_label_indexs, bb);
            // mask_rcnn_interface.create_semantic_objects(mask_rcnn_labels, mask_rcnn_label_indexs, bb, semantic_objects);

            if (mask_rcnn_success) {
                // mask_rcnn_mat.convertTo(mask_rcnn_mat, CV_32SC1);

                // tracker.assign_tracking_labels(semantic_objects,mask_rcnn_mat,tracked_mask, tracked_viz);
                // tracked_mask.convertTo(tracked_mask, CV_32SC1);


                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono8", mask_rcnn_mat).toImageMsg();
                maskrcnn_raw.publish(img_msg);

                img_msg = cv_bridge::CvImage(original_header, "rgb8", mask_rcnn_viz).toImageMsg();
                maskrcnn_viz.publish(img_msg);
                
            }
            else {
                ROS_WARN_STREAM("Could not analyse mask rcnn images");
            }
        }

        if (run_mono_depth) {
            mono_depth_success = mono_depth.analyse(current_image, mono_depth_mat);

            if (mono_depth_success) {
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono16", mono_depth_mat).toImageMsg();
                monodepth_raw.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mono depthimages");
            }
        }

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "rgb8", image).toImageMsg();
        input_image.publish(img_msg);

        previous_image = current_image;

    }


}