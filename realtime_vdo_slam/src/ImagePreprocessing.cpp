#include "ImagePreprocessing.hpp"
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vdo_slam.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

using namespace VDO_SLAM;

ImagePrepcoessing::ImagePrepcoessing(ros::NodeHandle& n) :
        handler(n),
        sceneflow(n),
        mask_rcnn_interface(n),
        mono_depth(n),
        image_transport(n),
        is_first(true),
        scene_flow_success(false),
        mask_rcnn_success(false),
        mono_depth_success(false)
{

    handler.param<std::string>("/input_camera_topic", input_camera_topic, "/camera/image_raw");
    handler.param<std::string>("/input_camera_info_topic", input_camera_info_topic, "/camera/camera_info");
    handler.param<bool>("/apply_undistortion", undistord_images, false);
    handler.param<bool>("/run_mask_rcnn", run_mask_rcnn, false);
    handler.param<bool>("/run_flow_net", run_scene_flow, false);
    handler.param<bool>("/run_mono_depth", run_mono_depth, false);

    if (undistord_images) {
        ROS_INFO_STREAM("Waiting for camera info topic: " << input_camera_info_topic);
        sensor_msgs::CameraInfoConstPtr info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_camera_info_topic);

        //for undistortion
        camera_info.camera_info_msg = *info;

        uint32_t image_width = camera_info.camera_info_msg.width;
        uint32_t image_height = camera_info.camera_info_msg.height;

        cv::Size image_size = cv::Size(image_width, image_height);

        if (camera_info.camera_info_msg.distortion_model == "rational_polynomial") {
            camera_info.camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info.camera_info_msg.K[0]);
            camera_info.dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info.camera_info_msg.D[0]);
        }
        else if (camera_info.camera_info_msg.distortion_model == "equidistant") {
            camera_info.camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info.camera_info_msg.K[0]);
            camera_info.dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info.camera_info_msg.D[0]);

            //cv::Mat scaled_camera_matrix = camera_matrix *
            // camera_info.camera_matrix.at<double>(2, 2) = 1.;

            //cv::Mat output_image;
            cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64F);

            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_info.camera_matrix, camera_info.dist_coeffs, image_size,
                                                                    identity_mat, camera_info.modified_camera_matrix);

            cv::fisheye::initUndistortRectifyMap(camera_info.camera_matrix,
                                                camera_info.dist_coeffs,
                                                identity_mat,
                                                camera_info.modified_camera_matrix,
                                                image_size,
                                                CV_16SC2,
                                                camera_info.map1, camera_info.map2);
        }
    }


    if(run_mask_rcnn) {
        //first we check if the services exist so we dont start them again
        if  (!ros::service::exists("mask_rcnn_service", true)) {
            ROS_INFO_STREAM("starting mask rcnn service");
            mask_rcnn_interface.start_service();
            ros::service::waitForService("mask_rcnn_service");
            MaskRcnnInterface::set_mask_labels(handler);
        }
        else {
            ROS_INFO_STREAM("Mask Rcnn already active");
        }
        MaskRcnnInterface::set_mask_labels(handler);
    }
    if(run_scene_flow) {
        if  (!ros::service::exists("flow_net_service", true)) { 
            ROS_INFO_STREAM("starting flow net service");
            sceneflow.start_service();
            ros::service::waitForService("flow_net_service");
        }
        else {
            ROS_INFO_STREAM("Flow Net already active");
        }
    }

    if(run_mono_depth) {
        if  (!ros::service::exists("mono_depth_service", true)) { 
            ROS_INFO_STREAM("starting mono_depth service");
            mono_depth.start_service();
            ros::service::waitForService("mono_depth_service");
        }
        else {
            ROS_INFO_STREAM("MonoDepth already active");
        }
    }

    ROS_INFO_STREAM("Input video topic " << input_camera_topic);



    image_subscriber = image_transport.subscribe(input_camera_topic, 20,
                                               &ImagePrepcoessing::image_callback, this);

    ///camera/rgb/image_raw
    //these will be published as the output of the program with the same timestamp as well as the
    //the original image
    input_image = image_transport.advertise("/vdoslam/input/camera/rgb/image_raw", 10);

    maskrcnn_raw = image_transport.advertise("/vdoslam/input/camera/mask/image_raw", 10);
    maskrcnn_viz = image_transport.advertise("/vdoslam/input/camera/mask/colour_mask", 10);

    flownet_raw = image_transport.advertise("/vdoslam/input/camera/flow/image_raw", 10);
    flownet_viz = image_transport.advertise("/vdoslam/input/camera/flow/colour_map", 10);
    monodepth_raw = image_transport.advertise("/vdoslam/input/camera/depth/image_raw", 10);

}

ImagePrepcoessing::~ImagePrepcoessing() {}

void ImagePrepcoessing::image_callback(const sensor_msgs::ImageConstPtr& msg) {
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
            scene_flow_success = sceneflow.analyse_image(current_image, previous_image, scene_flow_mat, scene_flow_viz);

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
            mask_rcnn_success = mask_rcnn_interface.analyse_image(current_image, mask_rcnn_mat, mask_rcnn_viz, mask_rcnn_labels, mask_rcnn_label_indexs);

            if (mask_rcnn_success) {
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
            mono_depth_success = mono_depth.analyse_image(current_image, mono_depth_mat);

            if (mono_depth_success) {
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "mono16", mono_depth_mat).toImageMsg();
                monodepth_raw.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mono depthimages");
            }
        }

        // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(original_header, "rgb8", image).toImageMsg();
        input_image.publish(msg);

        previous_image = current_image;

    }


}


void ImagePrepcoessing::undistortImage(cv::Mat& input, cv::Mat& undistorted) {
    if (camera_info.camera_info_msg.distortion_model == "rational_polynomial") {
        cv::undistort(input, undistorted, camera_info.camera_matrix, camera_info.dist_coeffs);
    }
    else if (camera_info.camera_info_msg.distortion_model == "equidistant") {
        cv::remap(input, undistorted, camera_info.map1, camera_info.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }

}

