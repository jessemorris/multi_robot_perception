#ifndef VDO_SLAM_SCENE_H
#define VDO_SLAM_SCENE_H

#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <string>
#include <vector>
#include <iostream>

namespace VDO_SLAM
{
    struct SceneObject {
        //these really should be cv::Mat so we can save them in SE(3) space - currently ignoring rotation
        cv::Point3f pose;
        cv::Point2f velocity;
        // cv::Mat center_image; //center in the 2D image plane in the form (u, v)
        int label_index; //this is semantic label
        std::string label;
        int tracking_id;


        SceneObject(const SceneObject& scene_object) :
            pose(scene_object.pose),
            velocity(scene_object.velocity),
            label_index(scene_object.label_index),
            // center_image(scene_object.center_image),
            label(scene_object.label),
            tracking_id(scene_object.tracking_id) {}

        SceneObject() {}

        //hack to make class polymorphic
        virtual void vf() {}
        friend std::ostream &operator << (std::ostream& output, const SceneObject& object);
        
    };

    class Scene {
        
        public:
            Scene(int _id);
            Scene(int _id, double _timestamp);
            // Scene(const Scene& scene) :
            //     id(scene.get_id()),
            //     timestamp(scene.get_timestamp()),
            //     scene_objects(scene.scene_objects),
            //     camera_pos_translation(scene.camera_pos_translation),
            //     camera_pos_rotation(scene.camera_pos_rotation),
            //     camera_vel_translation(scene.camera_vel_translation),
            //     camera_vel_rotation(scene.camera_vel_rotation) {}


            const cv::Point3f& camera_pos_T() const;
            const cv::Mat& camera_pos_R() const;

            const cv::Point3f& camera_vel_T() const;
            const cv::Mat& camera_vel_R() const;

            void add_scene_object(SceneObject _object);
            void update_camera_pos(cv::Mat& pos_matrix); //should be in form [R | t]
            void update_camera_vel(cv::Mat& vel_matrix); //should be in form [R | t]
            std::vector<SceneObject>& get_scene_objects();
            SceneObject* get_scene_objects_ptr();

            const int scene_objects_size();
            const int get_global_fid() const;
            const int get_id() const;
            const double get_timestamp() const;

        protected:
            std::vector<SceneObject> scene_objects;
            cv::Point3f camera_pos_translation;
            cv::Mat camera_pos_rotation; //should be 3x3 rotation matrix

            cv::Point3f camera_vel_translation;
            cv::Mat camera_vel_rotation; //should be 3x3 rotation matrix

        private:
            int global_fid;
            int id;
            double timestamp;
    };
    
} // namespace VDO_SLAM

#endif
