#include "vdo_slam/utils/Types.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/Converter.h"

#include <opencv2/opencv.hpp>

using namespace VDO_SLAM;


// std::ostream& operator<<(std::ostream& os, const VDO_SLAM::SceneType& scene) {
//     if (scene == SceneType::SINGLE)
//         os << "SINGLE"; 
//     else if (scene == SceneType::OPTIMIZED)
//         os << "OPTIMIZED";  
//     else if (scene == SceneType::ERROR)
//         os << "ERROR";
//     else
//         os << "?";

//     return os;
// }


void VDO_SLAM::EuclideanObject::pose_from_homogenous_mat(const cv::Mat& pose_) {
    assert(pose_.rows == pose_.cols == 4 && "Pose matrix should be in homogenous form");
    g2o::SE3Quat quat = VDO_SLAM::Converter::toSE3Quat(pose_);
    std::shared_ptr<g2o::SE3Quat> quat_ptr = std::make_shared<g2o::SE3Quat>(quat);
    pose = quat_ptr;
}

void VDO_SLAM::EuclideanObject::pose_from_vector(const cv::Mat& vector) {
    if(!utils::is_column_vector(vector)) {
        VDO_ERROR_MSG("Translation vector is not in column vector form [3x1]");
    }
    cv::Mat pose_hom = utils::homogenous_identity();
    pose_hom.at<double>(0, 3) = vector.at<double>(0, 0);
    pose_hom.at<double>(1, 3) = vector.at<double>(1, 0);
    pose_hom.at<double>(2, 3) = vector.at<double>(2, 0);
    pose_from_homogenous_mat(pose_hom);
}

void VDO_SLAM::EuclideanObject::twist_from_homogenous_mat(const cv::Mat& twist_) {
    assert(twist_.rows == twist_.cols == 4 && "Twist matrix should be in homogenous form");
    g2o::SE3Quat quat = VDO_SLAM::Converter::toSE3Quat(twist_);
    std::shared_ptr<g2o::SE3Quat> quat_ptr = std::make_shared<g2o::SE3Quat>(quat);
    twist = quat_ptr;

}

void VDO_SLAM::EuclideanObject::twist_from_vector(const cv::Mat& vector) {
    if(!utils::is_column_vector(vector)) {
        VDO_ERROR_MSG("Translation vector is not in column vector form [3x1]");
    }    
    cv::Mat twist_hom = utils::homogenous_identity();
    twist_hom.at<double>(0, 3) = vector.at<double>(0, 0);
    twist_hom.at<double>(1, 3) = vector.at<double>(1, 0);
    twist_hom.at<double>(2, 3) = vector.at<double>(2, 0);
    twist_from_homogenous_mat(twist_hom);

}
