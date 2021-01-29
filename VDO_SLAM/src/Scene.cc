#include "vdo_slam/Scene.h"




std::ostream &VDO_SLAM::operator << (std::ostream& output, const VDO_SLAM::SceneObject& object) {
    output << "SceneObject [pose:\nx: " << object.pose.x <<"\ny: " << object.pose.y;
    output << "\n Velocity:\nx: " << object.velocity.x <<"\ny: " << object.velocity.y;
    output << "\nLabel: " << object.label<< " Label index: " << object.label_index << " tracking ID " << object.tracking_id << " ]";

    return output;
}

VDO_SLAM::Scene::Scene(int _id, double _timestamp):
    id(_id),
    timestamp(_timestamp) {}

//I think I do want to copy here
void VDO_SLAM::Scene::add_scene_object(VDO_SLAM::SceneObject _object) {
    scene_objects.push_back(_object);
}
void VDO_SLAM::Scene::update_camera_pos(cv::Mat& pos_matrix) {
    //take translation part of matrix
    camera_pos_translation.x = pos_matrix.at<float>(0,3);
    camera_pos_translation.y = pos_matrix.at<float>(1,3);
    camera_pos_translation.z = pos_matrix.at<float>(2,3);

    //take rotation
    camera_pos_rotation = (cv::Mat_<float>(3,3) << pos_matrix.at<float>(0,0), pos_matrix.at<float>(0,1), pos_matrix.at<float>(0,2),
                                                   pos_matrix.at<float>(1,0), pos_matrix.at<float>(1,1), pos_matrix.at<float>(1,2),
                                                   pos_matrix.at<float>(2,0), pos_matrix.at<float>(2,1), pos_matrix.at<float>(2,2));
}

void  VDO_SLAM::Scene::update_camera_vel(cv::Mat& vel_matrix) {
    //take translation part of matrix
    camera_vel_translation.x = vel_matrix.at<float>(0,3);
    camera_vel_translation.y = vel_matrix.at<float>(1,3);
    camera_vel_translation.z = vel_matrix.at<float>(2,3);

    //take rotation
    camera_vel_rotation = (cv::Mat_<float>(3,3) << vel_matrix.at<float>(0,0), vel_matrix.at<float>(0,1), vel_matrix.at<float>(0,2),
                                                   vel_matrix.at<float>(1,0), vel_matrix.at<float>(1,1), vel_matrix.at<float>(1,2),
                                                   vel_matrix.at<float>(2,0), vel_matrix.at<float>(2,1), vel_matrix.at<float>(2,2));
}

std::vector<VDO_SLAM::SceneObject>& VDO_SLAM::Scene::get_scene_objects() {
    return scene_objects;
}

const int VDO_SLAM::Scene::scene_objects_size() {
    return scene_objects.size();
}

VDO_SLAM::SceneObject* VDO_SLAM::Scene::get_scene_objects_ptr() {
    return scene_objects.data();
}
 

const int& VDO_SLAM::Scene::get_global_fid() const {
    return global_fid;
}

const int& VDO_SLAM::Scene::get_id() const {
    return id;
}
const double& VDO_SLAM::Scene::get_timestamp() const {
    return timestamp;
}


