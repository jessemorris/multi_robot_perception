#include "SceneFlow.hpp"

#include <flow_net/FlowNet.h>
#include <cv_bridge/cv_bridge.h>



SceneFlow::SceneFlow(ros::NodeHandle& n) :
        nh(n) 
    {
      
    flow_net_client  = n.serviceClient<flow_net::FlowNet>("flow_net_service");

    }


bool SceneFlow::analyse_image(cv::Mat& current_image,cv::Mat& previous_image, cv::Mat& dst) {
    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();
    sensor_msgs::ImagePtr previous_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", previous_image).toImageMsg();

    flow_net::FlowNet srv;
    srv.request.previous_image = *previous_image_msg;
    srv.request.current_image = *current_image_msg;

    if(flow_net_client.call(srv)) {

        if (srv.response.success) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_image, sensor_msgs::image_encodings::RGB8);
            cv::Mat image = cv_ptr->image;

            //do i need to copy here?
            dst = image;
            return true;
        }
        else {
            ROS_ERROR_STREAM("Flow net service returned failed success");
            return false;
        }




    }
    else {
        ROS_ERROR_STREAM("Failed to call flow net service");
        return false;
    }




}