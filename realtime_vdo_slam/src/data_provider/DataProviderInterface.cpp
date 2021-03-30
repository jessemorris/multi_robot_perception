#include "data_provider/DataProviderInterface.hpp"

using namespace VDO_SLAM;

DataProviderInterface::DataProviderInterface()
    : nh("data_provider") {}

void DataProviderInterface::connect_slam_scene_callback(SlamSceneCallbackFunc&& func) {
    ROS_INFO_STREAM("Vdo Slam Scene callback connected");
    slam_scene_callback = func;
    slam_scene_connected = true;
}

void DataProviderInterface::connect_camera_info_callback(CameraInfoCallbackFunc&& func) {
    ROS_INFO_STREAM("Camera info callback connected");
    camera_info_callback = func;
    camera_info_connected = true;
}