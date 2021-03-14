#include "ImagePreprocessingRGB.hpp"
#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_preprocessing_rgb");
    ros::NodeHandle n;
    VDO_SLAM::preprocessing::ImageRGB image_processing(n);
    ros::spin();
}

