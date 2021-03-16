#ifndef _ROS_VDO_VISUALIZER
#define _ROS_VDO_VISUALIZER

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>

namespace VDO_SLAM {

    /**
     * @brief Overlays an image with all information from a Slam Scene message.
     * This includes bounding box, class, tracking ID, pose and velocity
     * 
     * @param image rgb image to overlay
     * @param slam_scene slam scene to details
     * @return cv::Mat overlayed image
     */
    cv::Mat overlay_scene_image(const cv::Mat& image, const realtime_vdo_slam::VdoSlamScenePtr& slam_scene);

}


#endif