#include "MaskRcnnInterface.hpp"

#include <mask_rcnn/MaskRcnnVisualise.h>
#include <mask_rcnn/MaskRcnnVdoSlam.h>
#include <mask_rcnn/MaskRcnnLabel.h>
#include <mask_rcnn/MaskRcnnLabelList.h>
#include <python_service_starter/StartMaskRcnn.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <iostream>


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
    mask_rcnn_labels = nh.serviceClient<mask_rcnn::MaskRcnnLabel>("mask_rcnn_label");
    mask_rcnn_labels_list = nh.serviceClient<mask_rcnn::MaskRcnnLabelList>("mask_rcnn_label_list");

    return service_started;
    
}

std::string MaskRcnnInterface::request_label(int index) {
    return mask_labels[index];
}

bool MaskRcnnInterface::request_labels(const std::vector<int>& label_indexs, std::vector<std::string>& labels) {
    labels.resize(0);
    if (mask_labels.size() > 0) {
        for (int label_index :label_indexs) {
            std::string label = mask_labels[label_index];
            labels.push_back(label);
        }
        return true;
    }
    else {
        ROS_WARN_STREAM("No labels in mask labels");
        return false;
    }
}

bool MaskRcnnInterface::set_mask_labels() {
    mask_rcnn::MaskRcnnLabelList all_labels;
    if (mask_rcnn_labels_list.call(all_labels)) {
        mask_labels = all_labels.response.labels;
        std::string labels_print;
        for (std::string& label : mask_labels) {
            labels_print += label += ", ";
        }
        ROS_INFO_STREAM("Setting mask labels: " << labels_print);
        return true;
    }
    return false;
}


bool MaskRcnnInterface::analyse_image(cv::Mat& current_image, cv::Mat& dst, 
    std::vector<std::string>& labels, std::vector<int>& label_indexs) {

    if (!service_started) {
        return false;
    }
    sensor_msgs::ImagePtr current_image_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", current_image).toImageMsg();

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