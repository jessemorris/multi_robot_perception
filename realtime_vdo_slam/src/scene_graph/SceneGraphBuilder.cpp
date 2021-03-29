#include "scene_graph/SceneGraphBuilder.hpp"

#include <functional>


using namespace VDO_SLAM;
// using std::placeholders::_1;

SceneGraphBuilder::SceneGraphBuilder()
   : nh("vdo_slam_scene_graph")
{
    ROS_INFO_STREAM("Using Ros Bag data provider");
    data_provider = std::make_shared<RosBagDataProvider>();
    data_provider->connect_callback(boost::bind(&SceneGraphBuilder::slam_scene_callback, this, _1));
    graph = std::make_shared<SceneGraph>();
}

void SceneGraphBuilder::load_data() {
    data_provider->spin();
}

void SceneGraphBuilder::slam_scene_callback(const realtime_vdo_slam::VdoSlamSceneConstPtr& scene) {
    slam_scenes.push_back(scene);    

}

void SceneGraphBuilder::construct_graph() {
    ROS_INFO_STREAM("Constructing graph with " << slam_scenes.size() << " slam messages");
    for (realtime_vdo_slam::VdoSlamSceneConstPtr& slam : slam_scenes) {
        realtime_vdo_slam::VdoSlamScene slam_scene = *slam;
        graph->add_dynamic_object(slam_scene);
    }

    auto map = graph->optimize_object_poses();
    // graph->show_optimized_poses(map, 4);
    graph->reconstruct_slam_scene(map);

}