#include "ImagePreprocessingRGBD.hpp"
#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_preprocessing_rgbd");
    ros::NodeHandle n;
    VDO_SLAM::preprocessing::ImageRGBD image_processing(n);
    ros::spin();
}

