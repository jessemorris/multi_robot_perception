#include "vdo_slam/Scene.h"
#include "vdo_slam/Converter.h"
#include "vdo_slam/Macros.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/utils/Types.h"
#include "vdo_slam/Map.h"


using namespace VDO_SLAM;



std::ostream &VDO_SLAM::operator << (std::ostream& output, const VDO_SLAM::SceneObject& object) {
    output << "SceneObject [pose:\n: " << *object.pose;
    output << "\n Velocity:\n: " << object.twist;
    output << "\nLabel: " << object.label<< " Semantic Instance: " << object.semantic_instance_index << " tracking ID " << object.tracking_id << " ]";

    return output;
}

VDO_SLAM::Scene::Scene()
{
    // camera_pos_translation.x = 0.0;
    // camera_pos_translation.y = 0.0;
    // camera_pos_translation.z = 0.0;

    // //take rotation
    // camera_pos_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
    //                                                 0, 1, 0,
    //                                                 0, 0, 1);
    cv::Mat identity = VDO_SLAM::utils::homogenous_identity();
    pose_from_homogenous_mat(identity);
    twist_from_homogenous_mat(identity);

}

VDO_SLAM::Scene::Scene(int frame_id_, double _timestamp):
    frame_id(frame_id_),
    timestamp(_timestamp) {
        //init camera pose
        // camera_pos_translation.x = 0.0;
        // camera_pos_translation.y = 0.0;
        // camera_pos_translation.z = 0.0;

        // //take rotation
        // camera_pos_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
        //                                                0, 1, 0,
        //                                                0, 0, 1);
        cv::Mat identity = VDO_SLAM::utils::homogenous_identity();
        pose_from_homogenous_mat(identity);
        twist_from_homogenous_mat(identity);
    }


//I think I do want to copy here
void VDO_SLAM::Scene::add_scene_object(SceneObjectPtr& _object) {
    _object->timestamp = timestamp;
    scene_objects.push_back(_object);
}

void Scene::update_pose_from_refined(const Map& map) {
    cv::Mat camera_pose = map.vmCameraPose_RF[frame_id];
    utils::image_to_global_coordinates(camera_pose, camera_pose);

    pose_from_vector(camera_pose);
}

std::vector<std::shared_ptr<SceneObject>>& VDO_SLAM::Scene::get_scene_objects() {
    return scene_objects;
}

int VDO_SLAM::Scene::scene_objects_size() {
    return scene_objects.size();
}

// VDO_SLAM::SceneObject* VDO_SLAM::Scene::get_scene_objects_ptr() {
//     return scene_objects.data();
// }
 
int VDO_SLAM::Scene::get_id() {
    return frame_id;
}
double VDO_SLAM::Scene::get_timestamp() {
    return timestamp;
}


