#include "vdo_slam/Scene.h"
#include "vdo_slam/Converter.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/utils/Types.h"
#include "vdo_slam/map/Map.h"
#include "vdo_slam/map/MapObject.h"



using namespace VDO_SLAM;



std::ostream &VDO_SLAM::operator << (std::ostream& output, const VDO_SLAM::SceneObject& object) {
    output << "SceneObject [pose:\n: " << *object.pose;
    output << "\n Velocity:\n: " << *object.twist;
    output << "\nLabel: " << object.label<< " Semantic Instance: " << object.semantic_instance_index << " tracking ID " << object.tracking_id << " ]";

    return output;
}


bool VDO_SLAM::SceneObject::update_from_map(const Map* map) {

    
}



VDO_SLAM::Scene::Scene()
{

    cv::Mat identity = VDO_SLAM::utils::homogenous_identity();
    pose_from_homogenous_mat(identity);
    twist_from_homogenous_mat(identity);

}

VDO_SLAM::Scene::Scene(int frame_id_, double _timestamp):
    frame_id(frame_id_),
    timestamp(_timestamp) {
    
        cv::Mat identity = VDO_SLAM::utils::homogenous_identity();
        pose_from_homogenous_mat(identity);
        twist_from_homogenous_mat(identity);
    }


void VDO_SLAM::Scene::add_scene_object(SceneObjectPtr& _object) {
    _object->timestamp = timestamp;
    _object->frame_id = frame_id;
    scene_objects.push_back(_object);
}

bool VDO_SLAM::Scene::update_from_map(const Map* map) {
    cv::Mat rf_camera_pose = map->vmCameraPose_RF[frame_id-1];
    VDO_INFO_MSG(rf_camera_pose);
    // utils::image_to_global_coordinates(rf_camera_pose, rf_camera_pose);
    pose_from_homogenous_mat(rf_camera_pose);
    return true;
}


std::vector<std::shared_ptr<SceneObject>>& VDO_SLAM::Scene::get_scene_objects() {
    return scene_objects;
}

int VDO_SLAM::Scene::scene_objects_size() {
    return scene_objects.size();
}

 
int VDO_SLAM::Scene::get_id() {
    return frame_id;
}
double VDO_SLAM::Scene::get_timestamp() {
    return timestamp;
}


