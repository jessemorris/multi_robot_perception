#include "visualizer/RosVisualizer.hpp"

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>


using namespace VDO_SLAM;


cv::Mat VDO_SLAM::overlay_scene_image(const cv::Mat& image, const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
    cv::Mat overlayed;
    image.copyTo(overlayed);


    std::vector<realtime_vdo_slam::VdoSceneObject> objects = slam_scene->objects;

    for(realtime_vdo_slam::VdoSceneObject& object : objects) {
        //draw bounding box
        cv::rectangle(overlayed, cv::Rect2d(object.bounding_box.center.x, object.bounding_box.center.y,
            object.bounding_box.size_x, object.bounding_box.size_y), 
            cv::Scalar(0, 255, 0), 2);


        //add info
        char text[200];
        sprintf(text, "%s, [%d] %02fkm/h", object.label.c_str(), object.tracking_id, object.twist.linear.x);
        cv::putText(overlayed, text, cv::Point(object.bounding_box.center.x, object.bounding_box.center.y), 
            cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

    }

    return overlayed;
}