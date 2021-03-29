#ifndef VDO_ROS_SCENE_GRAPH
#define VDO_ROS_SCENE_GRAPH

#include <vector>
#include <map>
#include <memory>

#include <realtime_vdo_slam/VdoSlamScene.h>


#include "scene_graph/SceneGraphOptimizer.hpp"




struct SlamObjectAssociation {
    ros::Time time;
    realtime_vdo_slam::VdoSceneObject object;
    realtime_vdo_slam::VdoSlamScene* slam_scene_ptr;

};

typedef int TrackingId;


typedef std::vector<SlamObjectAssociation> SlamObjectAssociationVector;
typedef std::map<TrackingId, SlamObjectAssociationVector> DynObjectMap;
    
typedef std::vector<realtime_vdo_slam::VdoSlamScene> SlamSceneVector;
typedef std::vector<realtime_vdo_slam::VdoSceneObject> SlamObjectsVector;

class SceneGraph {

    public:
        SceneGraph();

        // SlamObjectsVector get_current_objects();
        void add_dynamic_object(realtime_vdo_slam::VdoSlamScene& scene);
        void optimize_object_poses();

        

    private:
        ros::Time last_message_time;
        DynObjectMap dyn_object_map;


};

typedef std::shared_ptr<SceneGraph> SceneGraphPtr;
typedef std::unique_ptr<SceneGraph> SceneGraphUniquePtr;


#endif