#ifndef VDO_SLAM_SCENE_H
#define VDO_SLAM_SCENE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <memory>
#include <opencv2/core/core.hpp>

#include <eigen3/Eigen/Dense>
#include  <vdo_slam_g2o/types/types_seven_dof_expmap.h>

#include "vdo_slam/Types.h"

namespace VDO_SLAM
{

    struct SceneObject : public EuclideanObject {

        // g2o::SE3Quat pose;
        // g2o::SE3Quat twist;
        // cv::Point3f pose;
        // cv::Point2f velocity;
        cv::Mat center_image; //center in the 2D image plane in the form (u, v)
        int semantic_instance_index; 
        std::string label;
        int tracking_id; 
        int unique_id;

        //hack to make class polymorphic
        virtual void vf() {}

        friend std::ostream &operator << (std::ostream& output, const SceneObject& object);
        
    };

    class Scene : public EuclideanObject {
        
        public:
            Scene();
            Scene(int _id, double _timestamp);


            void add_scene_object(SceneObject _object);
            std::vector<SceneObject>& get_scene_objects();
            SceneObject* get_scene_objects_ptr();

            const int scene_objects_size();
            const int get_id() const;
            const double get_timestamp() const;

        protected:
            std::vector<SceneObject> scene_objects;
            // cv::Point3f camera_pos_translation;
            // cv::Mat camera_pos_rotation; //should be 3x3 rotation matrix

            // cv::Point3f camera_vel_translation;
            // cv::Mat camera_vel_rotation; //should be 3x3 rotation matrix
            // g2o::SE3Quat pose;
            // g2o::SE3Quat twist;

        private:
            int id;
            double timestamp;
    };

    
} // namespace VDO_SLAM

typedef std::shared_ptr<VDO_SLAM::SceneObject> VdoSlamSceneObjectPtr;
typedef std::unique_ptr<VDO_SLAM::SceneObject> VdoSlamSceneObjectUniquePtr;

typedef std::shared_ptr<VDO_SLAM::Scene> VdoSlamScenePtr;
typedef std::unique_ptr<VDO_SLAM::Scene> VdoSlamSceneUniquePtr;

#endif
