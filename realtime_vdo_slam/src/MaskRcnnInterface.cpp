#include "MaskRcnnInterface.hpp"

#include <mask_rcnn/MaskRcnnVisualise.h>
#include <mask_rcnn/MaskRcnnVdoSlam.h>
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
    mask_rcnn_client = nh.serviceClient<mask_rcnn::MaskRcnnVdoSlam>("mask_rcnn_service");

    return service_started;
    
}


bool MaskRcnnInterface::analyse_image(cv::Mat& current_image, cv::Mat& dst, 
    std::vector<std::string>& labels, std::vector<int>& label_indexs) {

    if (!service_started) {
        return false;
    }
    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", current_image).toImageMsg();

    mask_rcnn::MaskRcnnVdoSlam srv;
    srv.request.input_image = *current_image_msg;

    if(mask_rcnn_client.call(srv)) {

        if (srv.response.success) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.output_mask, sensor_msgs::image_encodings::MONO8);
            cv::Mat image = cv_ptr->image;

            for (std::vector<std::string>::iterator it = srv.response.labels.begin(); it != srv.response.labels.end(); ++it) {
                labels.push_back(*it);
            }

            for (std::vector<int>::iterator it = srv.response.label_indexs.begin(); it != srv.response.label_indexs.end(); ++it) {
                label_indexs.push_back(*it);
            }
                
            // labels.assign(std::begin(*srv.response.labels.data()), std::end(*srv.response.labels.data()));
            // label_indexs.assign(std::begin(*srv.response.label_indexs.data()), std::end(*srv.response.label_indexs.data()));

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