// #include "RosScene.hpp"

// #include <image_transport/image_transport.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/Transform.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <iostream>
// #include <stdio.h>
// #include <nav_msgs/Odometry.h>
// #include <opencv2/core.hpp>
// #include <vdo_slam/Scene.h>
// #include <vdo_slam/utils/Types.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <geometry_msgs/PoseWithCovariance.h>
// #include <opencv2/core.hpp>

// #include "utils/RosUtils.hpp"

// using namespace VDO_SLAM;
// /**
//  * @brief Converts a segmentation index (ie. a pixel level segmentation or the classification index
//  * assignment to a scene object) to a colour so each different classification can be visualisaed.
//  * 
//  * Uses the MaskRcnnInterface to get all the available categories and divides the RGB colour specitrum amongst
//  * them evenly.
//  * 
//  * @param index 
//  * @return cv::Scalar 
//  */
// cv::Scalar seg_index_to_colour(int index) {
//     ROS_INFO_STREAM(index);
//     // int total_categories = MaskRcnnInterface::categories_size();

//     //as a hack assume 20 categories so we can get nice clear colours
//     int total_categories = 20;
//     ROS_INFO_STREAM(total_categories);
//     int options = (255.0 * 3.0/(float)total_categories);

//     int r = 0;
//     int g = 0;
//     int b = 0;

//     int result = index * options;
//     ROS_INFO_STREAM("Result " << result);

//     if (result > 510) {
//         r = 255;
//         g = 255;
//         result -= 255;

//     }

//     if (result > 255) {
//         g = 255;
        
//         result -= 255;
//         b = result;
//     }

//     b = result;

//     return cv::Scalar(r, g, b);


// }




// //should also convert unix timestamp to ROS time
// //current timetstamp is just time difference and not unix time
// VDO_SLAM::RosSceneObject::RosSceneObject(realtime_vdo_slam::VdoSceneObjectConstPtr& _msg) : 
//     time(_msg->time) {

//         *pose = utils::g2o_converter::from_pose_msg(_msg->pose);
//         *twist = utils::g2o_converter::from_twist_msg(_msg->twist);

//         semantic_instance_index = _msg->semantic_label;
//         //this one should have lavel
//         //TODO: some assert if label not default - shoudl set this somehwere?
//         label = _msg->label;
//         tracking_id = _msg->tracking_id;
//         unique_id = _msg->uid;

//         vision_msgs::BoundingBox2D bb = _msg->bounding_box;
//         bounding_box = BoundingBox::create<vision_msgs::BoundingBox2D>(bb);


//     }

// VDO_SLAM::RosSceneObject::RosSceneObject(realtime_vdo_slam::VdoSceneObject& _msg):
//     time(_msg.time) {
//         *pose = utils::g2o_converter::from_pose_msg(_msg.pose);
//         *twist = utils::g2o_converter::from_twist_msg(_msg.twist);

//         semantic_instance_index = _msg.semantic_label;
//         //this one should have lavel
//         //TODO: some assert if label not default - shoudl set this somehwere?
//         label = _msg.label;
//         tracking_id = _msg.tracking_id;
//         unique_id = _msg.uid;
//         vision_msgs::BoundingBox2D bb = _msg.bounding_box;

//         bounding_box = BoundingBox::create<vision_msgs::BoundingBox2D>(bb);
// }


// VDO_SLAM::RosSceneObject::RosSceneObject(SceneObject& _object, ros::Time& _time) :
//     SceneObject(_object),
//     time(_time) {}

// realtime_vdo_slam::VdoSceneObjectPtr VDO_SLAM::RosSceneObject::to_msg() {
//     realtime_vdo_slam::VdoSceneObjectPtr msg(new realtime_vdo_slam::VdoSceneObject);
//     // msg->pose.position.x = pose.x;
//     // msg->pose.position.y = pose.y;
//     // msg->pose.position.z = pose.z;
//     // ROS_INFO_STREAM("making ros slam scene");
//     msg->pose = utils::g2o_converter::to_pose_msg(*pose);
//     msg->twist = utils::g2o_converter::to_twist_msg(*twist);

//     // msg->twist.linear.x = velocity.x;
//     // msg->twist.linear.y = velocity.y;
//     msg->semantic_label = semantic_instance_index;
//     msg->label = label;

//     //we should not label here becuase the scene object may not have the correct label
//     msg->tracking_id = tracking_id;
//     msg->time = time;
//     msg->uid = unique_id;
//     msg->bounding_box = bounding_box.convert<vision_msgs::BoundingBox2D>();


//     return msg;

// }

// std::ostream& VDO_SLAM::operator<<(std::ostream& os, const RosSceneObject& scene) {
//     os << "RosSceneObject\nPose: " << *scene.pose << "\nlabel: " << scene.label << "\ntracking id: " << scene.tracking_id;
//     return os;
// }

// VDO_SLAM::RosScene::RosScene(Scene& _object, ros::Time _time) :
//     time(_time),
//     Scene(_object) {

//         //Note: we dont add transform frame and child frame here
//         odom.header.stamp = time;
//         //we ignore covariance on odom here for now
//         odom.pose.pose = utils::g2o_converter::to_pose_msg(*pose);
//         //this only sets the linear velocity of the twist object as we only get
//         //linear from VDO
//         odom.twist.twist = utils::g2o_converter::to_twist_msg(*twist);

// }

// //Jesse: unsire how to convert timestamp into ros time becuase we're using
// //rostime not walltime
// VDO_SLAM::RosScene::RosScene(realtime_vdo_slam::VdoSlamSceneConstPtr& _msg)
//     :   time(_msg->header.stamp) {

//         frame_id = _msg->id;
//         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(_msg->original_frame, sensor_msgs::image_encodings::RGB8);
//         rgb_frame = cv_ptr->image;


//         *pose = utils::g2o_converter::from_pose_msg(_msg->camera_pose);
//         *twist = utils::g2o_converter::from_twist_msg(_msg->camera_twist);
        
//         for(const realtime_vdo_slam::VdoSceneObject& c_object : _msg->objects) {
//             realtime_vdo_slam::VdoSceneObject object = c_object;
//             // RosSceneObject ros_object(object);
//             RosSceneObjectPtr ros_object = std::make_shared<RosSceneObject>(object);
//             scene_objects.push_back(ros_object);

//         }


//     }

// const nav_msgs::Odometry& VDO_SLAM::RosScene::odom_msg() const {
//     return odom;
// }


// const ros::Time& VDO_SLAM::RosScene::get_ros_time() {
//     return time;
// }

// std::ostream& VDO_SLAM::operator<<(std::ostream& os, const VDO_SLAM::RosScene& scene) {
//     os << "\nRosScene\nPose: " << *scene.pose << "\nframe ID: " << scene.frame_id << "\nObjects (" << scene.scene_objects.size() << ")";
//     return os;
// }


// realtime_vdo_slam::VdoSlamScenePtr VDO_SLAM::RosScene::to_msg() {
//     realtime_vdo_slam::VdoSlamScenePtr msg(new realtime_vdo_slam::VdoSlamScene);
//     msg->camera_pose = utils::g2o_converter::to_pose_msg(*pose);
//     msg->camera_twist = utils::g2o_converter::to_twist_msg(*twist);


//     msg->id = frame_id;
//     for (SceneObjectPtr& object : scene_objects) {
//         ROS_INFO_STREAM(object);
//         RosSceneObjectPtr ros_object = std::make_shared<RosSceneObject>(*object, time);
//         // RosSceneObjectPtr ros_object = std::dynamic_pointer_cast<RosSceneObject>(object);
//         realtime_vdo_slam::VdoSceneObject object_msg = *ros_object->to_msg();
//         msg->objects.push_back(object_msg);

//     }
//     std_msgs::Header header = std_msgs::Header();
//     header.stamp = time;
//     utils::mat_to_image_msg(msg->original_frame, rgb_frame, sensor_msgs::image_encodings::RGB8, header);


//     return msg;
// }

// realtime_vdo_slam::VdoSlamMapPtr RosScene::make_map(std::vector<SlamScenePtr>& map) {
//     if (map.size() == 0) {
//         return nullptr;
//     }
//     else {
//         ROS_INFO_STREAM("reconstructing slam map for ROS " << map.size());
//         realtime_vdo_slam::VdoSlamMapPtr slam_map = boost::make_shared<realtime_vdo_slam::VdoSlamMap>();
//         for (SlamScenePtr& scene : map) {

//             //should not be time now but the time of the frame -> currenrly
//             //the frame timestamp is "timdiff" and unsure how to actually convert from
//             //a ISO timestamp to ROS time. Could keep a list of original timestamp
//             //and associate with the frames later -> assuming all the frames are correct
//             //when we reconstruct them from the map!
//             RosScene unique_ros_scene(*scene, ros::Time::now());
//             // ROS_INFO_STREAM(unique_ros_scene);
//             slam_map->scenes.push_back(*unique_ros_scene.to_msg());
//         }
//         return slam_map;
//     }
// }
