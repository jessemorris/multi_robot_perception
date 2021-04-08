#include "vdo_slam/Types.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/Converter.h"

#include <opencv2/opencv.hpp>


VDO_SLAM::eSensor VDO_SLAM::param_to_sensor(const int sensor) {
    if (sensor == 0) {
        VDO_INFO_MSG("Using MONOCULAR sensor.");
        return VDO_SLAM::eSensor::MONOCULAR;
    }
    else if (sensor == 1) {
        VDO_INFO_MSG("Using STEREO sensor.");
        return VDO_SLAM::eSensor::STEREO;
    }
    else if (sensor == 2) {
        VDO_INFO_MSG("Using RGBD sensor.");
        return VDO_SLAM::eSensor::RGBD;
    }
    else {
        VDO_ERROR_MSG("Sensor type " << sensor << " is invalid.");
        return VDO_SLAM::eSensor::INVALID;
    }

}

cv::Mat VDO_SLAM::homogenous_identity() {

    cv::Mat identity = (cv::Mat_<double>(4,4) << 1, 0, 0, 0,
                                                0, 1, 0, 0,
                                                0, 0, 1, 0,
                                                0, 0, 0, 1);
    return identity;
}


void VDO_SLAM::EuclideanObject::pose_from_cvmat(const cv::Mat& pose_) {
    assert(pose_.rows == pose_.cols == 4 && "Pose matrix should be in homogenous form");
    g2o::SE3Quat quat = VDO_SLAM::Converter::toSE3Quat(pose_);
    std::shared_ptr<g2o::SE3Quat> quat_ptr = std::make_shared<g2o::SE3Quat>(quat);
    pose = quat_ptr;



}

void VDO_SLAM::EuclideanObject::twist_from_cvmat(const cv::Mat& twist_) {
    assert(twist_.rows == twist_.cols == 4 && "Twist matrix should be in homogenous form");
    g2o::SE3Quat quat = VDO_SLAM::Converter::toSE3Quat(twist_);
    std::shared_ptr<g2o::SE3Quat> quat_ptr = std::make_shared<g2o::SE3Quat>(quat);
    twist = quat_ptr;

}