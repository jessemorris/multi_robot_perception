#include "RealTimeVdoSlam.hpp"

#include <sensor_msgs/CameraInfo.h>


RealTimeVdoSLAM::RealTimeVdoSLAM(ros::NodeHandle& n) :
        handler(n),
        sceneflow(n),
        image_transport(n),
        is_first(true)
{
    //set up config
    //TODO: param not working?
    handler.param<std::string>("/realtime_vdo_slam/topic_prefix", topic_prefix, "/gmsl/");
    handler.param<std::string>("/realtime_vdo_slam/camera_suffix", camera_suffix, "/image_color");
    handler.param<std::string>("/realtime_vdo_slam/info_msg_suffix", info_msg_suffix, "/camera_info");

    handler.param<std::string>("/realtime_vdo_slam/camera_selection", camera_selection, "A0");
    ROS_INFO_STREAM("camera selection " << camera_selection);

    // gmsl/<>/image_colour
    output_video_topic = topic_prefix + camera_selection + camera_suffix;
    camea_info_topic = topic_prefix + camera_selection + info_msg_suffix;

    ROS_INFO_STREAM("video topic " << output_video_topic);
    ROS_INFO_STREAM("camera info topic " << camea_info_topic);

        
    auto info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camea_info_topic, handler, ros::Duration(3));

    //this is so dumb but trying to avoid boost
    std::vector<double> camera_intrinsic_vector;

    //length of intrinsic matrix is 9
    for (int i = 0; i < 9; i++) {
        camera_intrinsic_vector.push_back(info->K[i]);
    }
    int count = 0;
    for(int i = 0; i < 3; i++) {
        std::vector<double> row;
        for(int j = 0; j < 3; j++) {
            row.push_back(camera_intrinsic_vector[count]);
            count++;
        }
        camera_intrinsic_matrix.push_back(row);
    }

    camera_distortion_matrix = info->D;
    ROS_INFO_STREAM("Received camera info");


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
    cv::Mat image;



    cv::undistort(distored, image, camera_intrinsic_matrix, camera_distortion_matrix);
    cv::Mat flow_matrix;

    if (is_first) {
        previous_image = image;
        is_first = false;
        return;
    }
    // else if(scene_flow_count >= scene_flow_count_max) {
    else {
        cv::Mat current_image = image;
        bool result = sceneflow.analyse_image(current_image, previous_image, flow_matrix);

        if (result) {
            std_msgs::Header header = std_msgs::Header();

            //TODO: proper headers
            header.frame_id = "base_link";
            header.stamp = ros::Time::now();
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", flow_matrix).toImageMsg();
            results.publish(img_msg);
        }
        else {
            ROS_WARN_STREAM("Could not analyse scene flow images");
        }


        previous_image = current_image.clone();


    }



}