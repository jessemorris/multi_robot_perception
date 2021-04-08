#include "vdo_slam/Scene.h"
#include "vdo_slam/Types.h"
#include "vdo_slam/Converter.h"
#include "vdo_slam/Macros.h"




std::ostream &VDO_SLAM::operator << (std::ostream& output, const VDO_SLAM::SceneObject& object) {
    output << "SceneObject [pose:\n: " << object.pose;
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
    cv::Mat identity = VDO_SLAM::homogenous_identity();
    pose_from_cvmat(identity);
    twist_from_cvmat(identity);

}

VDO_SLAM::Scene::Scene(int _id, double _timestamp):
    id(_id),
    timestamp(_timestamp) {
        //init camera pose
        // camera_pos_translation.x = 0.0;
        // camera_pos_translation.y = 0.0;
        // camera_pos_translation.z = 0.0;

        // //take rotation
        // camera_pos_rotation = (cv::Mat_<float>(3,3) << 1, 0, 0,
        //                                                0, 1, 0,
        //                                                0, 0, 1);
        cv::Mat identity = VDO_SLAM::homogenous_identity();
        pose_from_cvmat(identity);
        twist_from_cvmat(identity);
    }


//I think I do want to copy here
void VDO_SLAM::Scene::add_scene_object(VDO_SLAM::SceneObject _object) {
    scene_objects.push_back(_object);
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
 
const int VDO_SLAM::Scene::get_id() const {
    return id;
}
const double VDO_SLAM::Scene::get_timestamp() const {
    return timestamp;
}


