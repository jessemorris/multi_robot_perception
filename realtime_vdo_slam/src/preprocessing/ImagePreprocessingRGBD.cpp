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
    // if (depth->encoding == "16UC1"){
        sensor_msgs::Image img;
        img.header = depth->header;
        img.height = depth->height;
        img.width = depth->width;
        img.is_bigendian = depth->is_bigendian;
        img.step = depth->step;
        img.data = depth->data;
        img.encoding = "mono16";

        cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
        monodepth_raw.publish(cv_ptr_depth->toImageMsg());
	// }



    cv::Mat distored = cv_ptr_rgb->image;
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

    std_msgs::Header original_header = raw_image->header;

    std::vector<std::string> mask_rcnn_labels;
    std::vector<int> mask_rcnn_label_indexs;
    std::vector<vision_msgs::BoundingBox2D> bb;
    std::vector<mask_rcnn::SemanticObject> semantic_objects;

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


        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "rgb8", image).toImageMsg();
        input_image.publish(img_msg);

        previous_image = current_image;

    }

}