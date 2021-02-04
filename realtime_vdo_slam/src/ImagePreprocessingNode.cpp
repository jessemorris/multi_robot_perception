#include "ImagePreprocessing.hpp"
#include <ros/ros.h>


int main(int argc, char **argv) {
    ros::init(argc, argv, "image_preprocessing");
    ros::NodeHandle n;
    VDO_SLAM::ImagePrepcoessing image_processing(n);
    ros::spin();
}

