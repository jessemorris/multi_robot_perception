#include "CameraInformation.hpp"
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

VDO_SLAM::CameraInformation::CameraInformation(sensor_msgs::CameraInfoConstPtr& info_msg_ptr) {
    camera_info_msg = *info_msg_ptr;

    uint32_t image_width = camera_info_msg.width;
    uint32_t image_height = camera_info_msg.height;

    cv::Size image_size = cv::Size(image_width, image_height);

    if (camera_info_msg.distortion_model == "rational_polynomial") {
        camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info_msg.K[0]);
        dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info_msg.D[0]);

        distortion_model = DistortionModel::RAD_POLY;

    }
    else if (camera_info_msg.distortion_model == "equidistant") {
        camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info_msg.K[0]);
        dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info_msg.D[0]);

        //cv::Mat scaled_camera_matrix = camera_matrix *
        // camera_info.camera_matrix.at<double>(2, 2) = 1.;

        //cv::Mat output_image;
        cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64F);

        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, dist_coeffs, image_size,
                                                                identity_mat, modified_camera_matrix);

        cv::fisheye::initUndistortRectifyMap(camera_matrix,
                                            dist_coeffs,
                                            identity_mat,
                                            modified_camera_matrix,
                                            image_size,
                                            CV_16SC2,
                                            map1, map2);
        distortion_model = DistortionModel::EQUIDISTANT;
    }
    else {
        ROS_WARN_STREAM("Distortion Model [" << camera_info_msg.distortion_model << " not recognised.");
        distortion_model = DistortionModel::INVALID;
    }
}

void VDO_SLAM::CameraInformation::apply_undistortion(const cv::Mat& src, cv::Mat& dst) {
    if (distortion_model == DistortionModel::INVALID) {
        ROS_WARN_STREAM("Distortian model not set. Cannot undistort image");
        return;
    }

    if (camera_info_msg.distortion_model == "rational_polynomial") {
        cv::undistort(src, dst, camera_matrix, dist_coeffs);
    }
    else if (camera_info_msg.distortion_model == "equidistant") {
        cv::fisheye::undistortImage(src, dst, camera_matrix, dist_coeffs, modified_camera_matrix);
    }
}
