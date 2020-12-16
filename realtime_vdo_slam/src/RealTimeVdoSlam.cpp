#include "RealTimeVdoSlam.hpp"

#include <sensor_msgs/CameraInfo.h>
#include <vdo_slam.hpp>

RealTimeVdoSLAM::RealTimeVdoSLAM(ros::NodeHandle& n) :
        handler(n),
        sceneflow(n),
        mask_rcnn_interface(n),
        mono_depth(n),
        image_transport(n),
        is_first(true)
{
    //set up config
    //TODO: param not working?
    handler.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");
    handler.param<std::string>("/realtime_vdo_slam/camera_suffix", camera_suffix, "/image_color");
    handler.param<std::string>("/realtime_vdo_slam/info_msg_suffix", info_msg_suffix, "/camera_info");

    handler.param<std::string>("/realtime_vdo_slam/camera_selection", camera_selection, "A0");


    handler.param<bool>("/realtime_vdo_slam/apply_undistortion", undistord_images, false);
    handler.param<bool>("/realtime_vdo_slam/run_mask_rcnn", run_mask_rcnn, false);
    handler.param<bool>("/realtime_vdo_slam/run_flow_net", run_scene_flow, false);
    handler.param<bool>("/realtime_vdo_slam/run_mono_depth", run_mono_depth, false);

    if(run_mask_rcnn) {
        ROS_INFO_STREAM("starting mask rcnn service");
        mask_rcnn_interface.start_service();
    }
    if(run_scene_flow) {
        ROS_INFO_STREAM("starting flow net service");
        sceneflow.start_service();
    }

    if(run_mono_depth) {
        ROS_INFO_STREAM("starting mono_depth service");
        mono_depth.start_service();
    }

    ROS_INFO_STREAM("camera selection " << camera_selection);

    // gmsl/<>/image_colour
    output_video_topic = topic_prefix + camera_selection + camera_suffix;
    camea_info_topic = topic_prefix + camera_selection + info_msg_suffix;

    ROS_INFO_STREAM("video topic " << output_video_topic);
    ROS_INFO_STREAM("camera info topic " << camea_info_topic);

    camera_information.topic = output_video_topic;

    // if (undistord_images) {
    //     auto info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camea_info_topic, handler, ros::Duration(3));
    //     ROS_INFO_STREAM("Received camera info");

    //     camera_information.intrinsic = (cv::Mat_<double>(3,3) << info->K[0], info->K[1], info->K[2], 
    //                                                             info->K[3], info->K[4], info->K[5], 
    //                                                             info->K[6], info->K[7], info->K[8]);
    //     ROS_INFO_STREAM("Set camera intrinsics");

    //     camera_information.distortion = cv::Mat_<double>(1,info->D.size());
    //     memcpy(camera_information.distortion.data, info->D.data(), info->D.size()*sizeof(double));
    //     ROS_INFO_STREAM("Set camera distortion");
    //     ROS_INFO_STREAM("Un-distorting images " << undistord_images);

    // }

        

    

    // handler.param<int>("realtime_vdo_slam/scene_flow_delay_count", scene_flow_count_max, 5);

    image_subscriber = image_transport.subscribe(output_video_topic, 10,
                                               &RealTimeVdoSLAM::image_callback, this);

    results = image_transport.advertise("vdoslam/results", 10);
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
    cv::Mat flow_matrix, segmentation_mask;

    if (is_first) {
        previous_image = image;
        is_first = false;
        return;
    }
    else {
        cv::Mat current_image = image;
        if (run_scene_flow) {
            bool result = sceneflow.analyse_image(current_image, previous_image, flow_matrix);

            if (result) {
                std_msgs::Header header = std_msgs::Header();

                // //TODO: proper headers
                // header.frame_id = "base_link";
                // header.stamp = ros::Time::now();
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "rgb8", flow_matrix).toImageMsg();
                results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse scene flow images");
            }
        }

        if (run_mask_rcnn) {
            bool result = mask_rcnn_interface.analyse_image(current_image, segmentation_mask);

            if (result) {
                std_msgs::Header header = std_msgs::Header();

                // //TODO: proper headers
                // header.frame_id = "base_link";
                // header.stamp = ros::Time::now();
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "rgb8", segmentation_mask).toImageMsg();
                results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mask rcnn images");
            }
        }

        if (run_mono_depth) {
            bool result = mono_depth.analyse_image(current_image, mono_depth_mat);

            if (result) {
                std_msgs::Header header = std_msgs::Header();

                // //TODO: proper headers
                // header.frame_id = "base_link";
                // header.stamp = ros::Time::now();
                sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono16", mono_depth_mat).toImageMsg();
                results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mono depthimages");
            }
        }


        previous_image = current_image.clone();


    }



}