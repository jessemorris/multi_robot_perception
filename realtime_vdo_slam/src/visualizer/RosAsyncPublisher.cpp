#include "visualizer/RosAsyncPublisher.hpp"

using namespace VDO_SLAM;

RosAsyncPublisher::RosAsyncPublisher(RosCallbackQueuePtr& _callback_queue)
{
    callback_queue_ptr = _callback_queue;
}



void RosAsyncPublisher::connect_callback(const ros::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << "connected");
}
void RosAsyncPublisher::disconnect_callback(const ros::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << "unsubscribed");
}

void RosAsyncPublisher::connect_callback_image(const image_transport::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << "connected");
}
void RosAsyncPublisher::disconnect_callback_image(const image_transport::SingleSubscriberPublisher& pub) {
    std::string topic = pub.getTopic();
    ROS_INFO_STREAM(topic << "unsubscribed");
}