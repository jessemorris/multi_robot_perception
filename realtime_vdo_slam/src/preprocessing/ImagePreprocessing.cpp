#include "ImagePreprocessing.hpp"
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
        midas_depth(n),
        image_transport(n),
        is_first(true),
        scene_flow_success(false),
        mask_rcnn_success(false),
        mono_depth_success(false)
{
    handler.getParam("/vdo_preprocessing/rgb_topic", rgb_topic);
    handler.getParam("/vdo_preprocessing/depth_topic", depth_topic);
    handler.getParam("/vdo_preprocessing/seg_topic", seg_topic);
    handler.getParam("/vdo_preprocessing/flow_topic", flow_topic);
    handler.getParam("/vdo_preprocessing/rgb_info", rgb_info);

    input_type = get_input_type();

    if (input_type == InputType::INVALID) {
        ROS_ERROR_STREAM("Invalid input configuration.");
        ros::shutdown();
    }

    rgb_repub = image_transport.advertise("/vdoslam/input/camera/rgb/image_raw", 10);

    maskrcnn_raw = image_transport.advertise("/vdoslam/input/camera/mask/image_raw", 10);
    maskrcnn_viz = image_transport.advertise("/vdoslam/input/camera/mask/colour_mask", 10);

    flownet_raw = image_transport.advertise("/vdoslam/input/camera/flow/image_raw", 10);
    flownet_viz = image_transport.advertise("/vdoslam/input/camera/flow/colour_map", 10);
    monodepth_raw = image_transport.advertise("/vdoslam/input/camera/depth/image_raw", 10);

    vdo_input_pub = handler.advertise<realtime_vdo_slam::VdoInput>("/vdoslam/input/all", 10);

    //todo capture camera info topic


}

InputType BaseProcessing::get_input_type() {
    if (using_flow_topic() && using_seg_topic() && using_depth_topic() && using_rgb_topic()) {
        return InputType::RGB_DEPTH_SEG_FLOW;
    }
    else if (using_seg_topic() && using_depth_topic() && using_rgb_topic()) {
        return InputType::RGB_DEPTH_SEG;
    }
    else if (using_depth_topic() && using_rgb_topic()) {
        return InputType::RGB_DEPTH;
    }
    else if (using_rgb_topic()) {
        return InputType::RGB;
    }
    else {
        return InputType::INVALID;
    }
    

}

inline bool BaseProcessing::using_rgb_topic() {return !rgb_topic.empty()}
inline bool BaseProcessing::using_depth_topic() {return !depth_topic.empty()}
inline bool BaseProcessing::using_seg_topic() {return !seg_topic.empty()}
inline bool BaseProcessing::using_flow_topic() {return !flow_topic.empty()}



ImageRgb::ImageRgb(ros::NodeHandle& nh_)
    :   BaseProcessing(nh_) 
{
    
}
