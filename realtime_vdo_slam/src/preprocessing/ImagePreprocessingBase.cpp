#include "ImagePreprocessingBase.hpp"
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vdo_slam/vdo_slam.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

#include <mask_rcnn/SemanticObject.h>
#include <vision_msgs/BoundingBox2D.h>
#include <realtime_vdo_slam/VdoInput.h>
#include "CameraInformation.hpp"
#include <vdo_slam/System.h>
#include <vdo_slam/Types.h>

using namespace VDO_SLAM;
using namespace VDO_SLAM::preprocessing;


BaseProcessing::BaseProcessing(ros::NodeHandle& n) :
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
    int camera_input;
    handler.getParam("/vdo_preprocessing/camera_mode", camera_input);
    ROS_INFO_STREAM("Camera type is " << camera_input);

    camera_type = param_to_sensor(camera_input);

    handler.getParam("/vdo_preprocessing/color_topic", input_camera_topic);
    handler.getParam("/vdo_preprocessing/color_info", input_camera_info_topic);

    ROS_INFO_STREAM("Input video topic " << input_camera_topic);
    ROS_INFO_STREAM("Input video info " << input_camera_info_topic);


    if (camera_type == eSensor::RGBD) {
        handler.getParam("/vdo_preprocessing/depth_topic", input_depth_camera_topic);
        handler.getParam("/vdo_preprocessing/depth_info", input_camera_depth_topic);

        ROS_INFO_STREAM("Input depth topic " << input_depth_camera_topic);
        ROS_INFO_STREAM("Input depth info " << input_camera_depth_topic);


    }


    handler.param<bool>("/apply_undistortion", undistord_images, false);
    handler.param<bool>("/run_mask_rcnn", run_mask_rcnn, false);
    handler.param<bool>("/run_flow_net", run_scene_flow, false);
    handler.param<bool>("/run_mono_depth", run_mono_depth, false);


    if(run_mask_rcnn) {
        //first we check if the services exist so we dont start them again
        mask_rcnn_interface.start_service();
        mask_rcnn::MaskRcnnInterface::set_mask_labels(handler);
    }

    if(run_scene_flow) {
       sceneflow.start_service();
    }

    if(run_mono_depth && camera_type == eSensor::MONOCULAR) {
        mono_depth.start_service();
    }

    if (undistord_images) {
        ROS_INFO_STREAM("Waiting for camera info topic: " << input_camera_info_topic << " to undistort images.");
        sensor_msgs::CameraInfoConstPtr info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_camera_info_topic);

        camera_info = std::make_unique<CameraInformation>(info);
    }



    ///camera/rgb/image_raw
    //these will be published as the output of the program with the same timestamp as well as the
    //the original image
    input_image = image_transport.advertise("/vdoslam/input/camera/rgb/image_raw", 10);

    maskrcnn_raw = image_transport.advertise("/vdoslam/input/camera/mask/image_raw", 10);
    maskrcnn_viz = image_transport.advertise("/vdoslam/input/camera/mask/colour_mask", 10);

    flownet_raw = image_transport.advertise("/vdoslam/input/camera/flow/image_raw", 10);
    flownet_viz = image_transport.advertise("/vdoslam/input/camera/flow/colour_map", 10);
    monodepth_raw = image_transport.advertise("/vdoslam/input/camera/depth/image_raw", 10);

    vdo_input_pub = handler.advertise<realtime_vdo_slam::VdoInput>("/vdoslam/input/all", 10);

}

BaseProcessing::~BaseProcessing() {}


//TODO: use camerainformation class


void BaseProcessing::undistortImage(cv::Mat& input, cv::Mat& undistorted) {
    camera_info->apply_undistortion(input, undistorted);
}

