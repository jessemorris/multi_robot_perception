#include "RealTimeVdoSlam.hpp"
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <vdo_slam.hpp>

#include <memory>


RealTimeVdoSLAM::RealTimeVdoSLAM(ros::NodeHandle& n) :
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
    //set up config
    //TODO: param not working?
    handler.param<std::string>("/topic_prefix", topic_prefix, "/gmsl/");
    handler.param<std::string>("/camera_suffix", camera_suffix, "/image_color");
    handler.param<std::string>("/info_msg_suffix", info_msg_suffix, "/camera_info");

    handler.param<std::string>("/camera_selection", camera_selection, "A0");


    handler.param<bool>("/apply_undistortion", undistord_images, false);
    handler.param<bool>("/run_mask_rcnn", run_mask_rcnn, false);
    handler.param<bool>("/run_flow_net", run_scene_flow, false);
    handler.param<bool>("/run_mono_depth", run_mono_depth, false);

    //after how many images full batch oprtimisation should be run
    handler.param<int>("/global_optim_trigger", global_optim_trigger, 10);
    ROS_INFO_STREAM("global optim trigger " << global_optim_trigger);

    if(run_mask_rcnn) {
        ROS_INFO_STREAM("starting mask rcnn service");
        mask_rcnn_interface.start_service();
        ros::service::waitForService("mask_rcnn_service");
    }
    if(run_scene_flow) {
        ROS_INFO_STREAM("starting flow net service");
        sceneflow.start_service();
        ros::service::waitForService("flow_net_service");
    }

    if(run_mono_depth) {
        ROS_INFO_STREAM("starting mono_depth service");
        mono_depth.start_service();
        ros::service::waitForService("mono_depth_service");
    }

    ROS_INFO_STREAM("camera selection " << camera_selection);

    // gmsl/<>/image_colour
    output_video_topic = topic_prefix + camera_selection + camera_suffix;
    camea_info_topic = topic_prefix + camera_selection + info_msg_suffix;

    ROS_INFO_STREAM("video topic " << output_video_topic);
    ROS_INFO_STREAM("camera info topic " << camea_info_topic);

    camera_information.topic = output_video_topic;
    ROS_INFO_STREAM("Waiting for camera info msg");
    sensor_msgs::CameraInfoConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camea_info_topic);
    camera_information.camera_info = *camera_info;
    ROS_INFO_STREAM("Got camera info msg");


    image_subscriber = image_transport.subscribe(output_video_topic, 1,
                                               &RealTimeVdoSLAM::image_callback, this);

    maskrcnn_results = image_transport.advertise("vdoslam/results/maskrcnn", 10);
    flownet_results = image_transport.advertise("vdoslam/results/flownet", 10);
    monodepth_results = image_transport.advertise("vdoslam/results/monodepth", 10);


    std::string path = ros::package::getPath("realtime_vdo_slam");
    std::string vdo_slam_config_path = path + "/config/vdo_config.yaml";
    // VDO_SLAM::System SLAM(vdo_slam_config_path,VDO_SLAM::System::RGBD);
    slam_system = std::make_unique< VDO_SLAM::System>(vdo_slam_config_path,VDO_SLAM::System::RGBD);
    image_trajectory = cv::Mat::zeros(800, 600, CV_8UC3);
}

RealTimeVdoSLAM::~RealTimeVdoSLAM() {}

void RealTimeVdoSLAM::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat distored = cv_ptr->image;
    // cv::cvtColor(distored, distored, CV_RGB2BGR);
    cv::Mat image = distored;


    // if (undistord_images) {
    //     cv::undistort(distored, image, camera_information.intrinsic, camera_information.distortion);
    // }
    // else {
    //     image = distored;
    // }
    // cv::Mat flow_matrix, segmentation_mask;
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
        if (run_scene_flow) {
            scene_flow_success = sceneflow.analyse_image(current_image, previous_image, scene_flow_mat);

            if (scene_flow_success) {
                //#TODO: cannot fisplay until convert from scene flow to rgb
                // std_msgs::Header header = std_msgs::Header();

               
                // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "rgb8", scene_flow_mat).toImageMsg();
                // flownet_results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse scene flow images");
            }
        }

        if (run_mask_rcnn) {
            mask_rcnn_success = mask_rcnn_interface.analyse_image(current_image, mask_rcnn_mat, mask_rcnn_labels, mask_rcnn_label_indexs);

            if (mask_rcnn_success) {
                std_msgs::Header header = std_msgs::Header();

                // //TODO: proper headers
                // header.frame_id = "base_link";
                // header.stamp = ros::Time::now();
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono8", mask_rcnn_mat).toImageMsg();
                maskrcnn_results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mask rcnn images");
            }
        }

        if (run_mono_depth) {
            mono_depth_success = mono_depth.analyse_image(current_image, mono_depth_mat);

            if (mono_depth_success) {
                std_msgs::Header header = std_msgs::Header();

                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono16", mono_depth_mat).toImageMsg();
                monodepth_results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mono depthimages");
            }
        }


        previous_image = current_image.clone();
        previous_time = previous_time;


        //run slam algorithm
        //not we have no ground truth
        //times are just relative to each other so we can just record rostim between each callback
        // if (scene_flow_success && mask_rcnn_success && mono_depth_success) {
        //     ros::Duration diff = current_time - previous_time;
        //     double time_difference = diff.toSec();

        //     cv::Mat depth_image_float;
        //     cv::Mat ground_truth = cv::Mat::eye(4,4,CV_32F);
        //     std::vector<std::vector<float> > object_pose_gt;
        //     mono_depth_mat.convertTo(depth_image_float, CV_32F);
        //     mask_rcnn_mat.convertTo(mask_rcnn_mat, CV_32SC1);

        //     slam_system->TrackRGBD(image,depth_image_float,
        //         scene_flow_mat,
        //         mask_rcnn_mat,
        //         ground_truth,
        //         object_pose_gt,
        //         time_difference,
        //         image_trajectory,global_optim_trigger);
        // }


    }



}