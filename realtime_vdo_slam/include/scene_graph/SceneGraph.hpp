#ifndef VDO_ROS_SCENE_GRAPH
#define VDO_ROS_SCENE_GRAPH

#include <vector>
#include <map>
#include <memory>

#include <realtime_vdo_slam/VdoSlamScene.h>


#include "scene_graph/SceneGraphOptimizer.hpp"
#include <minisam/core/Factor.h>
#include <minisam/core/LossFunction.h>



struct SlamObjectAssociation {
    ros::Time time;
    realtime_vdo_slam::VdoSceneObject object;
    realtime_vdo_slam::VdoSlamScenePtr slam_scene_ptr;

};

typedef int TrackingId;


typedef std::vector<SlamObjectAssociation> SlamObjectAssociationVector;
typedef std::map<TrackingId, SlamObjectAssociationVector> DynObjectMap;
    
typedef std::vector<realtime_vdo_slam::VdoSlamScene> SlamSceneVector;
typedef std::vector<realtime_vdo_slam::VdoSceneObject> SlamObjectsVector;


//m and c curve parameters from the optimization
// where m and c define the function y=exp(mx+c)
typedef std::pair<double, double> CurveParamPair;

typedef std::pair<ros::Time, realtime_vdo_slam::VdoSlamScenePtr> SlamSceneTimePair;

class SceneGraph {

    public:
        SceneGraph();

        void add_dynamic_object(realtime_vdo_slam::VdoSlamScenePtr& scene);

        void optimize_object_poses(std::map<TrackingId, CurveParamPair>& optimized_map);

        void show_optimized_poses(std::map<TrackingId, CurveParamPair>& optimized_poses, int random_samples = -1);

        std::vector<realtime_vdo_slam::VdoSlamScenePtr>& reconstruct_slam_scene(std::map<TrackingId, CurveParamPair>& optimized_poses);

        

    private:

        ros::Time last_message_time;
        DynObjectMap dyn_object_map;
        std::shared_ptr<minisam::LossFunction> loss;

        std::vector<realtime_vdo_slam::VdoSlamScenePtr> slam_scenes;


};

typedef std::shared_ptr<SceneGraph> SceneGraphPtr;
typedef std::unique_ptr<SceneGraph> SceneGraphUniquePtr;


#endif