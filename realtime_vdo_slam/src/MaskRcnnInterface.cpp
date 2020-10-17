#include "MaskRcnnInterface.hpp"

#include <mask_rcnn/MaskRcnn.h>
#include <cv_bridge/cv_bridge.h>



MaskRcnnInterface::MaskRcnnInterface(ros::NodeHandle& n) :
        nh(n)
    {
      
    mask_rcnn_client  = n.serviceClient<mask_rcnn::MaskRcnn>("mask_rcnn_service");

    }


bool MaskRcnnInterface::analyse_image(cv::Mat& current_image, cv::Mat& dst) {
    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();

    mask_rcnn::MaskRcnn srv;
    srv.request.input_image = *current_image_msg;

    if(mask_rcnn_client.call(srv)) {

        if (srv.response.success) {
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