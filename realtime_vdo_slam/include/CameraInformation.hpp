#ifndef _ROS_VDO_CAMERA_INFORMATION
#define _ROS_VDO_CAMERA_INFORMATION

#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <memory>


namespace VDO_SLAM {

    enum class DistortionModel {
        NONE,
        RADTAN,
        RAD_POLY,
        EQUIDISTANT,
        INVALID
    };


    //wrapper for camera information
    struct CameraInformation {

        std::string topic;
        cv::Mat camera_matrix;
        cv::Mat dist_coeffs;

        //This will be P (see OpenCV docs for estimateNewCameraMatrixForUndistortRectify)
        cv::Mat map1, map2, modified_camera_matrix;
        sensor_msgs::CameraInfo camera_info_msg;
        DistortionModel distortion_model;

        CameraInformation() {}
        CameraInformation(sensor_msgs::CameraInfoConstPtr& info_msg_ptr);
        void apply_undistortion(const cv::Mat& src, cv::Mat& dst);

    };

    typedef std::unique_ptr<CameraInformation> CameraInformationPtr;

}


#endif
