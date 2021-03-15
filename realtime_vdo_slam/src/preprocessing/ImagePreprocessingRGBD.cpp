#include "ImagePreprocessingRGBD.hpp"


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

#include <vision_msgs/BoundingBox2D.h>
#include <mask_rcnn/SemanticObject.h>
#include <realtime_vdo_slam/VdoInput.h>

// #incl
#include <mono_depth_2/MonoDepthInterface.hpp>
#include <mask_rcnn/MaskRcnnInterface.hpp>
#include <mask_rcnn/SemanticTracker.hpp>
#include <flow_net/FlowNetInterface.hpp>

using namespace VDO_SLAM::preprocessing;

ImageRGBD::ImageRGBD(ros::NodeHandle& n):
    BaseProcessing(n),
    raw_img(n,input_camera_topic, 100),
    depth_img(n,input_depth_camera_topic, 100),
    sync(raw_img, depth_img, 100)
{
    sync.registerCallback(boost::bind(&ImageRGBD::image_callback, this, _1, _2));
}

void ImageRGBD::image_callback(ImageConst raw_image, ImageConst depth) {
    cv_bridge::CvImagePtr cv_ptr_rgb = cv_bridge::toCvCopy(*raw_image, sensor_msgs::image_encodings::RGB8);

    //we assume we can do this as the VDO pipeline expects the format to be in MONO16
    cv_bridge::CvImagePtr cv_ptr_depth = convert_img_msg(depth, sensor_msgs::image_encodings::MONO16);
    monodepth_raw.publish(cv_ptr_depth->toImageMsg());



    cv::Mat distored = cv_ptr_rgb->image;
    cv::Mat undistorted;
    
    cv::Mat image;
    if (undistord_images) {
        undistortImage(distored, undistorted);
        image = undistorted;
    }
    else {
        image = distored;
    }

    std_msgs::Header original_header = raw_image->header;

    std::vector<mask_rcnn::SemanticObject> semantic_objects;
    realtime_vdo_slam::VdoInput input_msg;

    input_msg.rgb = *raw_image;
    input_msg.depth = *depth;

    cv::Mat tracked_mask;
    cv::Mat tracked_viz;


    if (is_first) {
        previous_image = image;
        previous_time = raw_image->header.stamp;
        is_first = false;
        return;
    }
    else {
        cv::Mat current_image = image;
        current_time = raw_image->header.stamp;

        if (run_scene_flow) {
            scene_flow_success = sceneflow.analyse(current_image, previous_image, scene_flow_mat, scene_flow_viz);

            if (scene_flow_success) {
                //#TODO: cannot fisplay until convert from scene flow to rgb               
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "32FC2", scene_flow_mat).toImageMsg();
                flownet_raw.publish(img_msg);
                input_msg.flow = *img_msg;

                img_msg = cv_bridge::CvImage(original_header, "rgb8", scene_flow_viz).toImageMsg();
                flownet_viz.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse scene flow images");
            }
        }

        if (run_mask_rcnn) {
            mask_rcnn_success = mask_rcnn_interface.analyse(current_image, mask_rcnn_mat, mask_rcnn_viz, semantic_objects, current_time);

            if (mask_rcnn_success) {

                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono8", mask_rcnn_mat).toImageMsg();
                maskrcnn_raw.publish(img_msg);
                input_msg.mask = *img_msg;
                input_msg.semantic_objects = semantic_objects;

                img_msg = cv_bridge::CvImage(original_header, "rgb8", mask_rcnn_viz).toImageMsg();
                maskrcnn_viz.publish(img_msg);
                
            }
            else {
                ROS_WARN_STREAM("Could not analyse mask rcnn images");
            }
        }


        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "rgb8", image).toImageMsg();
        input_image.publish(img_msg);

        vdo_input_pub.publish(input_msg);

        previous_image = current_image;

    }

}