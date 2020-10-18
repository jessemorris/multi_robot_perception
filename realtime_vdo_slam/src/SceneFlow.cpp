#include "SceneFlow.hpp"

#include <flow_net/FlowNet.h>
#include <cv_bridge/cv_bridge.h>
#include <python_service_starter/StartFlowNet.h>



SceneFlow::SceneFlow(ros::NodeHandle& n) :
        nh(n),
        service_started(false)
    {
      
    flow_net_client  = nh.serviceClient<flow_net::FlowNet>("flow_net_service");
    flow_net_start = nh.serviceClient<python_service_starter::StartFlowNet>("start_flow_net");

    }

bool SceneFlow::start_service() {
    python_service_starter::StartFlowNet srv;
    srv.request.start = true;

    service_started = flow_net_start.call(srv);
    ROS_INFO_STREAM("Start flow net service returned " << service_started);
    return service_started;
    
}


bool SceneFlow::analyse_image(cv::Mat& current_image,cv::Mat& previous_image, cv::Mat& dst) {

    if (!service_started) {
        return false;
    }

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