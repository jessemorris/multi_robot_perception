#ifndef SEMANTIC_TRACKER_HPP
#define SEMANTIC_TRACKER_HPP

    // ROS Headers
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <mask_rcnn/SemanticObject.h>
#include <mask_rcnn/HungarianSolver.hpp>


#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <python_service_starter/ServiceStarterInterface.hpp>


#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <vision_msgs/BoundingBox2D.h>
#include <geometry_msgs/Pose2D.h>

// Standard Library Headers
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <deque>


namespace mask_rcnn {

    class SemanticTracker {

        public:
            SemanticTracker(const int _queue_size = 1);
            /**
             * @brief Assign tracking label to each semantic object in the current_list and modify the 
             * semantic matrix (which will be semantically labelled, eg class ) such that each pixel of the semantic
             * mask is now the tracking label so that the tracking will be consistent between frames. The current
             * semantic object list is compared to the most recent semantic object in the semanitc history queue. 
             * 
             * @param current_semantic_objects 
             * @param original_semantic_mat 
             * @param reassigned_semantic_mat 
             * @param viz 
             * @return true 
             * @return false 
             */
            bool assign_tracking_labels(std::vector<mask_rcnn::SemanticObject>& current_semantic_objects,
                                        const cv::Mat& original_semantic_mat, cv::Mat& reassigned_instance_mat, cv::Mat& viz);


        private:
            const int queue_size;
            HungarianAlgorithm hungarian_solver;
            int global_track_id = 1;

            std::deque<std::vector<mask_rcnn::SemanticObject>> semantic_object_history;

            /**
             * @brief Adds a semantic list object to the semantic history queue. If the queue is greater than the queue size
             * the oldest semantic object list be will thrown out. 
             * 
             * @param semantic_list A list of semantic objects. Assign tracking lables will compare aginst this list
             * if it is the most recent. 
             * @return true 
             * @return false 
             */
            bool add_semantic_objects(std::vector<mask_rcnn::SemanticObject>& semantic_list);
            
    };

}

#endif

