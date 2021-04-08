#ifndef VDO_SLAM_TYPES_H
#define VDO_SLAM_TYPES_H

#include <opencv2/opencv.hpp>
#include <vdo_slam_g2o/types/types_seven_dof_expmap.h>
#include <memory>



namespace VDO_SLAM {

     // Input sensor
    enum eSensor {
        MONOCULAR=0,
        STEREO=1, //confusing between stereo and RGBD, for now use RGBD?
        RGBD=2,
        INVALID=3
    };

    /**
     * @brief Converts an integer to its associated eSensor type
     * 
     * MONOCULAR = 0
     * STEREO = 1
     * RGBD = 2
     * INVALID = 3 (this will be returned if sensor is not 0, 1 or 2.)
     * 
     * @param sensor 
     * @return eSensor 
     */
    eSensor param_to_sensor(const int sensor);

    /**
     * @brief Constructs a 4x4 matrix in the homogenous form (R|t) where R
     * is the identity matrix and t is a 0 column vector
     * 
     * @return cv::Mat 
     */
    cv::Mat homogenous_identity();

      /**
     * @brief Defines a base class that contains information to
     * describe a physical object in 3D space. Uses the g2o::SE3Quat object
     * to define translational and rotational vectors. 
     * 
     * Includes conversion functions from cv::Mat's in homogenous matrix form. 
     * 
     */
    struct EuclideanObject {

        //TODO: should be unique ptr's but we make shared so so that we can use the 
        //default copy constructor
        std::shared_ptr<g2o::SE3Quat> pose;
        std::shared_ptr<g2o::SE3Quat> twist;

        /**
         * @brief Translation vector in pose
         * 
         * @return const Eigen::Vector3d& 
         */
        inline const Eigen::Vector3d& poseT() {return pose->translation();}

        /**
         * @brief Quaterion vector in pose
         * 
         * @return const Eigen::Quaterniond& 
         */
        inline const Eigen::Quaterniond& poseR() {return pose->rotation();}
        /**
         * @brief Translation vector in twist
         * 
         * @return const Eigen::Vector3d& 
         */
        inline const Eigen::Vector3d& twistT() {return twist->translation();}
        /**
         * @brief Quaternion vector in twist
         * 
         * @return const Eigen::Quaterniond& 
         */
        inline const Eigen::Quaterniond& twistR() {return twist->rotation();}

        /**
         * @brief Sets the pose object from a cv::Mat. The matrix should be in homogenous 
         * ([R | t]) form.
         * 
         * @param const cv::Mat&  pose 
         */
        void pose_from_cvmat(const cv::Mat& pose);
        /**
         * @brief Sets the twist object from a cv::Mat. The matrix should be in homogenous 
         * ([R | t]) form.
         * 
         * @param const cv::Mat& twist 
         */
        void twist_from_cvmat(const cv::Mat& twist);

    };
};


#endif
