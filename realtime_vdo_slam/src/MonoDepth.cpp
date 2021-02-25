#include "MonoDepth.hpp"

#include <mono_depth_2/MonoDepth.h>
#include <cv_bridge/cv_bridge.h>
#include <python_service_starter/StartMonoDepth.h>



MonoDepth::MonoDepth(ros::NodeHandle& n) :
        nh(n),
        service_started(false)
    {
      
    mono_depth_start = nh.serviceClient<python_service_starter::StartMonoDepth>("start_mono_depth");

    }

bool MonoDepth::start_service() {
    python_service_starter::StartMonoDepth srv;
    srv.request.start = true;

    service_started = mono_depth_start.call(srv);
    ROS_INFO_STREAM("Start Mono Depth service returned " << service_started);
    mono_depth_client  = nh.serviceClient<mono_depth_2::MonoDepth>("mono_depth_service");

    return service_started;
    
}

bool MonoDepth::wait_for_mono_services(ros::Duration timeout) {
    return ros::service::waitForService("mono_depth_service", timeout);
}


bool MonoDepth::analyse_image(cv::Mat& current_image, cv::Mat& dst) {

    if (!service_started) {
        return false;
    }

    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();

    mono_depth_2::MonoDepth srv;
    srv.request.current_image = *current_image_msg;

    if(mono_depth_client.call(srv)) {

        if (srv.response.success) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_image, sensor_msgs::image_encodings::MONO16);
            cv::Mat image = cv_ptr->image;

            //do i need to copy here?
            dst = image;
            return true;
        }
        else {
            ROS_ERROR_STREAM("Mono Depth service returned failed success");
            return false;
        }
        
    }
    else {
        ROS_ERROR_STREAM("Failed to call Mono Depth service");
        return false;
    }




}