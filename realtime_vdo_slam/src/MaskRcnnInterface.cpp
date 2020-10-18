#include "MaskRcnnInterface.hpp"

#include <mask_rcnn/MaskRcnn.h>
#include <python_service_starter/StartMaskRcnn.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>


//TOOD put in utils file for PytonServicesInterface
MaskRcnnInterface::MaskRcnnInterface(ros::NodeHandle& n) :
        nh(n),
        service_started(false)
    {
      
    mask_rcnn_start = nh.serviceClient<python_service_starter::StartMaskRcnn>("start_mask_rcnn");

    }

bool MaskRcnnInterface::start_service() {
    python_service_starter::StartMaskRcnn srv;
    srv.request.start = true;

    service_started = mask_rcnn_start.call(srv);
    ROS_INFO_STREAM("Start masrk rcnn service returned " << service_started);
    //must be initalised after call
    mask_rcnn_client = nh.serviceClient<mask_rcnn::MaskRcnn>("mask_rcnn_service");

    return service_started;
    
}


bool MaskRcnnInterface::analyse_image(cv::Mat& current_image, cv::Mat& dst) {

    if (!service_started) {
        return false;
    }
    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();

    mask_rcnn::MaskRcnn srv;
    srv.request.input_image = *current_image_msg;

    if(mask_rcnn_client.call(srv)) {

        if (srv.response.success) {
            ROS_INFO_STREAM("success");
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_image, sensor_msgs::image_encodings::RGB8);
            cv::Mat image = cv_ptr->image;

            //do i need to copy here?
            dst = image;
            return true;
        }
        else {
            ROS_ERROR_STREAM("Mask rcnn service returned failed success");
            return false;
        }
        
    }
    else {
        ROS_ERROR_STREAM("Failed to call Mask rcnn service");
        return false;
    }




}