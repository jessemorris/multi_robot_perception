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
void VDO_SLAM::Scene::update_camera_pos(float x, float y, float z) {
    camera_pos.x = x;
    camera_pos.y = y;
    camera_pos.z = z;
}

void  VDO_SLAM::Scene::update_camera_vel(float x, float y, float z) {
    camera_vel.x = x;
    camera_vel.y = y;
    camera_vel.z = z;
}

std::vector<VDO_SLAM::SceneObject>& VDO_SLAM::Scene::get_scene_objects() {
    return scene_objects;
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


