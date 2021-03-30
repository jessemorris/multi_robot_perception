#include "data_provider/RosBagDataProvider.hpp"
#include <rosbag/view.h>

using namespace VDO_SLAM;

RosBagDataProvider::RosBagDataProvider()
    :   DataProviderInterface()
{
    nh.param<std::string>("bag_name", bag_file_name, "invalid");
    nh.param<std::string>("scene_topic", scene_topic, "scene");
    nh.param<std::string>("camera_info_topic", camera_info_topic, "camera_info");

    topics.push_back(scene_topic);
    topics.push_back(camera_info_topic);

    if (bag_file_name != "invalid") {
        ROS_INFO_STREAM("Loading bagfile from file: " << bag_file_name);
        bag.open(bag_file_name, rosbag::bagmode::Read);
    }
    else {
        ROS_INFO_STREAM("Invalid bag file name");
    }
}

bool RosBagDataProvider::spin() {
    if (!slam_scene_connected) {
        ROS_ERROR_STREAM("Slam Scene func has not been connected");
        return false;
    }

    if (!camera_info_connected) {
        ROS_ERROR_STREAM("Camera info func has not been connected");
        return false;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m: view) {
        const std::string& msg_topic = m.getTopic();

        realtime_vdo_slam::VdoSlamSceneConstPtr msg = m.instantiate<realtime_vdo_slam::VdoSlamScene>();
        if (msg != nullptr && msg_topic == scene_topic) {
            slam_scene_callback(msg);
        } 

        sensor_msgs::CameraInfoConstPtr cam_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
        if (cam_info_msg != nullptr && msg_topic == camera_info_topic) {
            camera_info_callback(cam_info_msg);
        } 

        ros::spinOnce();

    }
    bag.close();
    return true;

}

