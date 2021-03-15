#ifndef _ROS_VDO_SLAM_INPUT
#define _ROS_VDO_SLAM_INPUT

#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>

#include <mask_rcnn/SemanticObject.h>

namespace VDO_SLAM {

    // /**
    //  * @brief We need the semantic mask and semantic objects for data association so we wrap them
    //  * 
    //  */
    // struct SemanticWrapper {
    //     std::vector<mask_rcnn::SemanticObject> semantic_objects;
    //     cv::Mat mask;
    // }

    /**
     * @brief Defines a wrapper that holds the all the required input to the VDO algorithm. Requires RGB image,
     * optical flow, depth map and semantic instance masking as well as image time.
     * 
     */
    struct VdoSlamInput {
        cv::Mat raw, flow, depth, mask;
        std::vector<std::vector<float> > object_pose_gt;
        cv::Mat ground_truth;
        double time_diff;
        ros::Time image_time; //when the image was created so we can keep track of the real time despite algorithmic delays


        /**
         * @brief Construct a new Vdo Slam Input object
         * 
         * @param _raw RGB input image. This should be the original image parsed to the vdo slam pre-processing node.
         * @param _flow Optical flow mat. This is the output of the flow_net node.
         * @param _depth Dense depth mat. This is the output of the mono_depth_2 node.
         * @param _mask Semantic instance masking. This is the output of the mask_rcnn node.
         * @param _time_diff The time (in nano seconds) between this frame and the previous frame.
         * @param _image_time The ros::Time the original image was captured (not the time it was processed). This is used
         * to synchronize the data streams as the VDO algorithm runs at a much slower frequency than a usual camera data stream. 
         */
        VdoSlamInput(cv::Mat& _raw, cv::Mat& _flow, cv::Mat& _depth, cv::Mat& _mask, double _time_diff, ros::Time& _image_time) : 
            raw(_raw),
            flow(_flow),
            depth(_depth),
            time_diff(_time_diff),
            image_time(_image_time)

        {
            ground_truth = cv::Mat::eye(4,4,CV_32F);
            // _depth.convertTo(depth, CV_32F);
            // _depth.convertTo(depth, CV_16UC1);
            _mask.convertTo(mask, CV_32SC1);
        }

    };

}



#endif
