#include <mask_rcnn/SemanticTracker.hpp>
#include <ros/ros.h>

#include <vision_msgs/BoundingBox2D.h>



using namespace mask_rcnn;

double get_euclid_distance(const vision_msgs::BoundingBox2D& b1, const vision_msgs::BoundingBox2D& b2) {
    //get centroid
    double center_x_b1 = b1.size_x/2.0;
    double center_y_b1 = b1.size_y/2.0;

    double center_x_b2 = b2.size_x/2.0;
    double center_y_b2 = b2.size_y/2.0;

    double dx = center_x_b1 - center_x_b2;
    double dy = center_y_b1 - center_y_b2;

    return std::sqrt((dx * dx) + (dy * dy));
}

SemanticTracker::SemanticTracker(const int _queue_size) :
    queue_size(_queue_size) {}

bool SemanticTracker::add_semantic_objects(std::vector<mask_rcnn::SemanticObject>& semantic_list) {

    semantic_object_history.push_back(semantic_list);

    if (semantic_object_history.size() > queue_size) {
        semantic_object_history.pop_front();
    }
    return true;
}

bool SemanticTracker::assign_tracking_labels(std::vector<mask_rcnn::SemanticObject>& current_semantic_objects,
                                        const cv::Mat& original_semantic_mat, cv::Mat& reassigned_instance_mat, cv::Mat& viz) {

    reassigned_instance_mat = cv::Mat::zeros(original_semantic_mat.rows, original_semantic_mat.cols, CV_8UC1);
    viz = cv::Mat::zeros(original_semantic_mat.rows, original_semantic_mat.cols, CV_8UC3);
                                
    if (semantic_object_history.empty()) {
        ROS_WARN_STREAM("Semantic history is empty. Please add a semantic list before tracking.");

        if (current_semantic_objects.empty()) {
            return false;
        }

        for(int i = 0; i < current_semantic_objects.size(); i++) {
            current_semantic_objects[i].tracking_label = global_track_id;
            ROS_INFO_STREAM(current_semantic_objects[i].tracking_label);
            global_track_id++;
        }
        add_semantic_objects(current_semantic_objects);

        return false;
    }
    else {

        //we want to solve for rows so create matrix of current semantic objects down and previous semantic objects in cols
        //need to get euclid distance between each object

        //get the last element we put in
        int previous_semantic_length = semantic_object_history.back().size(); //cols
        int current_semantic_length = current_semantic_objects.size(); //rows


        if (previous_semantic_length == 0 || current_semantic_length == 0) {
            return false;
        }

        mask_rcnn::SemanticObject* previous_semantic_objects = semantic_object_history.back().data();

        mask_rcnn::SemanticObject* prev_semantics_counter = previous_semantic_objects;


        //do I now want to pop back?
        std::vector<double> v(previous_semantic_length);

        std::vector<std::vector<double>> euclid_cost_matrix(current_semantic_length,v);
        for (int i = 0; i < current_semantic_length; i++) {
            std::vector<double> cols;
            for (int j = 0; j < previous_semantic_length; j++) {
                double cost = get_euclid_distance(current_semantic_objects[i].bounding_box, prev_semantics_counter->bounding_box);
                euclid_cost_matrix[i][j] = cost;
                prev_semantics_counter++;
            }

            //reset pointer
            prev_semantics_counter = previous_semantic_objects;

        }

        std::vector<int> assignment;
        std::cout << "H solution" << std::endl;
        double cost = hungarian_solver.Solve(euclid_cost_matrix, assignment);
        for (unsigned int current_track_index = 0; current_track_index < euclid_cost_matrix.size(); current_track_index++) {
            int assigned_track = assignment[current_track_index];

            if (assigned_track < 0) {
                //i guess keep own tracking label but this will be wrong? need new tracking label somehow
                current_semantic_objects[current_track_index].tracking_label = global_track_id;
                global_track_id++;
                continue;
            }

            //if previous track was not provided but we have an assignment - make new track
            if((previous_semantic_objects+assigned_track)->tracking_label == -1) {
                (previous_semantic_objects+assigned_track)->tracking_label = global_track_id;
                global_track_id++;
            }

            current_semantic_objects[current_track_index].tracking_label = (previous_semantic_objects+assigned_track)->tracking_label;

            //now modify mask and viz
            vision_msgs::BoundingBox2D bounding_box = current_semantic_objects[current_track_index].bounding_box;
            //in area round bounding box change any non-zero pixels to the tracking label. 
            //do same for viz mat but make it colourful!
            int left = bounding_box.center.x;
            int top = bounding_box.center.y;

            int width = bounding_box.size_x;
            int height = bounding_box.size_y;

            int x_end = left + width;
            int y_end = top + height;

            int y_end = std::min(y_end, original_semantic_mat.cols);
            int x_end = std::min(x_end, original_semantic_mat.rows);

            for (int rows = top; rows < y_end; rows++) {
                for (int cols = left; cols < x_end; cols++) {

                    if (original_semantic_mat.at<int>(rows, cols) != 0) {
                        // std::cout << original_semantic_mat.at<int>(rows, cols) << " ";

                        reassigned_instance_mat.at<int>(rows, cols) =  (uint16_t)current_semantic_objects[current_track_index].tracking_label * 50;
                        ROS_INFO_STREAM("Assigned track " << reassigned_instance_mat.at<int>(rows, cols));
                        viz.at<cv::Vec3b>(rows, cols)[0] = 0;
                        viz.at<cv::Vec3b>(rows, cols)[1] = current_semantic_objects[current_track_index].tracking_label * 50;
                        viz.at<cv::Vec3b>(rows, cols)[2] = 255;
                    }
                    else {
                        reassigned_instance_mat.at<int>(rows, cols) = 0;
                    }

                }

            }

            // cv::rectangle(viz, cv::Point(bounding_box.center.x, bounding_box.center.y),
            //         cv::Point(bounding_box.center.x + bounding_box.size_x, bounding_box.center.y + bounding_box.size_y),
            //         cv::Scalar(0, 255, 255), 2);


            // std::cout << current_track_index << "," << assignment[current_track_index] << "\t";

        }

        add_semantic_objects(current_semantic_objects);

    }



}