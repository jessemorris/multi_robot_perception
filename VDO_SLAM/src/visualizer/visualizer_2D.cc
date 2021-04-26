#include "vdo_slam/visualizer/visualizer_2D.h"
#include "vdo_slam/utils/VdoUtils.h"

namespace VDO_SLAM {

    Visualizer2D::Visualizer2D(VisualizerParamsPtr& params_)
        :   VisualizerBase(params_) {
            
            output_viz_ = make_unique<VisualizerOutput2D>();
            output_viz_->object_point_display_ = cv::Mat::zeros(800, 800, CV_8UC3);
        }

    VisualizerOutputUniquePtr Visualizer2D::spinOnce(SlamScenePtr& slam_scene_) {
        update_object_points_display(slam_scene_);
        update_projected_box_display(slam_scene_);

        VisualizerOutputUniquePtr result = make_unique<VisualizerOutput2D>(*output_viz_);
        return std::move(result);


    }

    void Visualizer2D::update_object_points_display(SlamScenePtr& slam_scene_) {
        display_lock.lock();
        

        double x = slam_scene_->poseT()[0];
        double y = slam_scene_->poseT()[1];
        double z = slam_scene_->poseT()[2];

        int x_display =  static_cast<int>(x*params->scale) + params->x_offset;
        int y_display =  static_cast<int>(y*params->scale) + params->y_offset;

        //add odom to cv mat
        cv::rectangle(output_viz_->object_point_display_, cv::Point(x_display, y_display), cv::Point(x_display+10, y_display+10), cv::Scalar(0,0,255),1);
        cv::rectangle(output_viz_->object_point_display_, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), CV_FILLED);
        cv::putText(output_viz_->object_point_display_, "Camera Trajectory (RED SQUARE)", cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
        char text[100];
        sprintf(text, "x = %02fm y = %02fm z = %02fm", x, y, z);
        cv::putText(output_viz_->object_point_display_, text, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

        std::vector<SceneObjectPtr> scene_objects = slam_scene_->get_scene_objects();
        for(SceneObjectPtr& scene_object : scene_objects) {
            double x = scene_object->poseT()[0];
            double y = scene_object->poseT()[1];

            // ros::Time t = scene->header.stamp;


            int x_display =  static_cast<int>(x*params->scale) + params->x_offset;
            int y_display =  static_cast<int>(y*params->scale) + params->y_offset;
            int track = scene_object->tracking_id;

            HashableColor color = color_manager_.get_colour_for_tracking_id(track);
            cv::circle(output_viz_->object_point_display_, cv::Point(x_display, y_display), 2, color, 5);


        }

        display_lock.unlock();

    }

    void Visualizer2D::update_gt_odom(g2o::SE3Quat& odom_gt_) {
        display_lock.lock();

        double x = odom_gt_.translation()[0];
        double y = odom_gt_.translation()[1];
        double z = odom_gt_.translation()[2];
       
        double x_display = static_cast<int>(x*params->scale) + params->x_offset;
        double y_display = static_cast<int>(y*params->scale) + params->y_offset;
        //add odom to cv mat
        cv::rectangle(output_viz_->object_point_display_, cv::Point(y_display, x_display), cv::Point(y_display+10, x_display+10), cv::Scalar(0,255,0),1);
        cv::rectangle(output_viz_->object_point_display_, cv::Point(10, 100), cv::Point(550, 130), CV_RGB(0,0,0), CV_FILLED);
        cv::putText(output_viz_->object_point_display_, "Camera GT Trajectory (GREEN SQUARE)", cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
        char text1[100];
        sprintf(text1, "x = %.2f y = %.2f z = %.2f", x, y, z);
        cv::putText(output_viz_->object_point_display_, text1, cv::Point(10, 120), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

        display_lock.unlock();
    }


    void Visualizer2D::update_projected_box_display(SlamScenePtr& slam_scene_) {
        cv::Mat overlayed = overlay_scene_image(slam_scene_);
        output_viz_->bounding_box_display_ = overlayed;

    }

    cv::Mat Visualizer2D::overlay_scene_image(const SlamScenePtr& slam_scene_) {

        if (slam_scene_ == nullptr) {
            return cv::Mat();
        }

        cv::Mat overlayed;
        slam_scene_->rgb_frame(overlayed);


        for(SceneObjectPtr& object : slam_scene_->get_scene_objects()) {
            //draw bounding box
            HashableColor color = color_manager_.get_colour_for_tracking_id(object->tracking_id);

            cv::rectangle(overlayed, cv::Rect2d(object->bounding_box.x, object->bounding_box.y,
                object->bounding_box.width, object->bounding_box.height), 
                color, 2);


            //add info
            char text[200];
            sprintf(text, "%s, [%d] %02fkm/h", object->label.c_str(), object->tracking_id, object->twistT()[0]);
            cv::putText(overlayed, text, cv::Point(object->bounding_box.x, object->bounding_box.y), 
                cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

        }

        return overlayed;
    
    }

}