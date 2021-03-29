#include "scene_graph/DataProviderInterface.hpp"

using namespace VDO_SLAM;

DataProviderInterface::DataProviderInterface()
    : nh("data_provider") {}

void DataProviderInterface::connect_callback(SlamSceneCallbackFunc&& func) {
    ROS_INFO_STREAM("Vdo Slam Scene callback connected");
    callback = func;
    is_connected = true;
}