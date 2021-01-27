#ifndef VDO_SLAM_SCENE_H
#define VDO_SLAM_SCENE_H

#include <opencv2/core/types.hpp>
#include <string>
#include <vector>
#include <iostream>

namespace VDO_SLAM
{
    struct SceneObject {
        cv::Point3f pose;
        cv::Point2f velocity;
        int label_index;
        std::string label;


        SceneObject(const SceneObject& scene_object) :
            pose(scene_object.pose),
            velocity(scene_object.velocity),
            label_index(scene_object.label_index),
            label(scene_object.label) {}

        SceneObject() {}
        friend std::ostream &operator << (std::ostream& output, const SceneObject& object);
        
    };

    class Scene {
        
        public:

            Scene(int _id, double _timestamp);
            Scene(int _id, double _timestamp);
            void add_scene_object(SceneObject& _object);
            void update_camera_pos(float x, float y, float z);
            std::vector<SceneObject>& get_scene_objects();

            const int& get_global_fid() const;
            const int& get_id() const;
            const double& get_timestamp() const;

        protected:
            std::vector<SceneObject> scene_objects;
            cv::Point3f camera_pos;

        private:
            int global_fid;
            int id;
            double timestamp;
    };
    
} // namespace VDO_SLAM

#endif
