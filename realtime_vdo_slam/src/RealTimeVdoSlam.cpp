#include "RealTimeVdoSlam.hpp"
#include "MaskRcnnInterface.hpp"
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vdo_slam.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

int VDO_SLAM::RosSceneManager::vis_count = 0;

//should also convert unix timestamp to ROS time
//current timetstamp is just time difference and not unix time
VDO_SLAM::RosSceneObject::RosSceneObject(SceneObject& _object, ros::Time& _time, int _uid) :
    SceneObject(_object),
    time(_time),
    uid(_uid) {}



VDO_SLAM::RosSceneManager::RosSceneManager(ros::NodeHandle& _nh) :
        nh(_nh)
    {
        visualiser = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/visualization", 20 );
        odom_pub = nh.advertise<nav_msgs::Odometry>("vdoslam/odom", 20);
        nh.getParam("/frame_id", child_frame_id);
    }

void VDO_SLAM::RosSceneManager::display_scene(RosScenePtr& scene) {
    const nav_msgs::Odometry odom = scene->odom_msg();
    const geometry_msgs::TransformStamped tf = scene->tf_transform_msg();

    visualization_msgs::MarkerArray marker_array;
    scene->make_vizualisation(marker_array);

    odom_pub.publish(odom);
    visualiser.publish(marker_array);
    broadcaster.sendTransform(tf);

}

VDO_SLAM::RosScene::RosScene(Scene& _object, ros::Time _time) :
    time(_time),
    Scene(_object) {

        ROS_INFO_STREAM("Camera pos: " << camera_pos_translation);
        ROS_INFO_STREAM("Camera rot: " << camera_pos_rotation);

        //convert them all into RosSceneObjects
        int id = 0;
        for(int i = 0; i < scene_objects.size(); i++) {
            RosSceneObject ros_scene_object(scene_objects[i], time, id);
            scene_objects[i] = ros_scene_object;
            id++;
        }


        //update tf pose for ROS
        transform_stamped.header.stamp = time;
        transform_stamped.header.frame_id = "vdo_odom";
        //TODO: want this to be param eventaully but lazy design currently and lack of time to refactor
        transform_stamped.child_frame_id = "vdo_camera_link";
        transform_stamped.transform.translation.x = -camera_pos_translation.x;
        transform_stamped.transform.translation.y = -camera_pos_translation.y;
        transform_stamped.transform.translation.z = 0;

        tf2::Quaternion quat;
        ROS_INFO_STREAM("making transform");
        tf2::Matrix3x3 rotation_matrix_pos(camera_pos_rotation.at<float>(0, 0), camera_pos_rotation.at<float>(0, 1), camera_pos_rotation.at<float>(0, 2),
                                           camera_pos_rotation.at<float>(1, 0), camera_pos_rotation.at<float>(1, 1), camera_pos_rotation.at<float>(1, 2),
                                           camera_pos_rotation.at<float>(2, 0), camera_pos_rotation.at<float>(2, 1), camera_pos_rotation.at<float>(2, 2));
                

        rotation_matrix_pos.getRotation(quat);
        
        //must provide quaternion!
        transform_stamped.transform.rotation.x = quat.x();
        transform_stamped.transform.rotation.y = quat.y();
        transform_stamped.transform.rotation.z = quat.z();
        transform_stamped.transform.rotation.w = quat.w();

        // ROS_INFO_STREAM("Published transform");
        odom.header.stamp = time;
        odom.header.frame_id = "vdo_odom";
        odom.child_frame_id = "vdo_camera_link";
        odom.pose.pose.position.x = -camera_pos_translation.x;
        odom.pose.pose.position.y = -camera_pos_translation.y;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        odom.twist.twist.linear.x = camera_vel_translation.x;
        odom.twist.twist.linear.y = camera_vel_translation.y;
        odom.twist.twist.linear.z = camera_vel_translation.z;

        //TODO: angular velocity

    }
const nav_msgs::Odometry& VDO_SLAM::RosScene::odom_msg() const {
    return odom;
}
const geometry_msgs::TransformStamped& VDO_SLAM::RosScene::tf_transform_msg() const {
    return transform_stamped;
}

void VDO_SLAM::RosScene::make_vizualisation(visualization_msgs::MarkerArray& marker_array) {
    for (SceneObject& scene_object: scene_objects) {
        ROS_INFO_STREAM(scene_object);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "vdo_odom";
        marker.header.stamp = time;
        marker.ns = "vdoslam";
        marker.id = scene_object.tracking_id;
        // marker.type = visualization_msgs::Marker::SPHERE;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = -scene_object.pose.x;
        marker.pose.position.y = -scene_object.pose.y;
        marker.pose.position.z = 0;



        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        marker.text = scene_object.label;

        marker_array.markers.push_back(marker);
        // vis_count++;
    }

}

RosVdoSlam::RosVdoSlam(ros::NodeHandle& n) :
        handle(n),
        ros_scene_manager(handle),
        mask_rcnn_interface(n),
        raw_img(handle,"/camera/rgb/image_raw", 5),
        mask_img(handle,"/camera/mask/image_raw", 5),
        flow_img(handle,"/camera/flow/image_raw", 5),
        depth_img(handle,"/camera/depth/image_raw", 5),
        sync(raw_img, mask_img, flow_img, depth_img, 10)

    {
        handle.getParam("/global_optim_trigger", global_optim_trigger);
        ROS_INFO_STREAM("Global Optimization Trigger at frame id: " << global_optim_trigger);

        //first we check if the services exist so we dont start them again
        if  (!ros::service::exists("mask_rcnn_service", true)) {
            ROS_INFO_STREAM("starting mask rcnn service");
            mask_rcnn_interface.start_service();
            ros::service::waitForService("mask_rcnn_service");
            MaskRcnnInterface::set_mask_labels(handle);
        }
        else {
            ROS_INFO_STREAM("Mask Rcnn already active");
        }
        MaskRcnnInterface::set_mask_labels(handle);

        std::string path = ros::package::getPath("realtime_vdo_slam");
        std::string vdo_slam_config_path = path + "/config/vdo_config.yaml";

        //TODO: should get proper previous time
        previous_time = ros::Time::now();
        image_trajectory = cv::Mat::zeros(800, 600, CV_8UC3);
        vdo_worker_thread = std::thread(&RosVdoSlam::vdo_worker, this);

        sync.registerCallback(boost::bind(&RosVdoSlam::vdo_input_callback, this, _1, _2, _3, _4));

        slam_system = std::make_unique< VDO_SLAM::System>(vdo_slam_config_path,VDO_SLAM::System::RGBD);


    }

RosVdoSlam::~RosVdoSlam() {
    if (vdo_worker_thread.joinable()) {
        vdo_worker_thread.join();
    }
}

void RosVdoSlam::vdo_input_callback(ImageConst raw_image, ImageConst mask, ImageConst flow, ImageConst depth) {
    //the actual time the image was craeted
    current_time = raw_image->header.stamp;
    ros::Duration diff = current_time - previous_time;
    //time should be in n seconds or seconds (or else?)
    double time_difference = diff.toNSec();

    cv::Mat image, scene_flow_mat, mono_depth_mat, mask_rcnn_mat;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(*raw_image, sensor_msgs::image_encodings::RGB8);
    image = cv_ptr->image;

    cv_ptr = cv_bridge::toCvCopy(*mask, sensor_msgs::image_encodings::MONO8);
    mask_rcnn_mat = cv_ptr->image;

    cv_ptr = cv_bridge::toCvCopy(*flow, sensor_msgs::image_encodings::TYPE_32FC2);
    scene_flow_mat = cv_ptr->image;

    cv_ptr = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::MONO16);
    mono_depth_mat = cv_ptr->image;



    std::shared_ptr<VdoSlamInput> input = std::make_shared<VdoSlamInput>(image,scene_flow_mat,
            mono_depth_mat, mask_rcnn_mat, time_difference, current_time);

    //add the input to the thread queue so we can deal with it later
    push_vdo_input(input);
    previous_time = current_time;
    
}


void RosVdoSlam::set_scene_labels(std::unique_ptr<VDO_SLAM::Scene>& scene) {
    int size = scene->scene_objects_size();
    VDO_SLAM::SceneObject* object_ptr = scene->get_scene_objects_ptr();
    ROS_DEBUG_STREAM("Updating scene labels (size " << size << ")");
    for(int i = 0; i < size; i++) {
        std::string label = MaskRcnnInterface::request_label(object_ptr->label_index);
        object_ptr->label = label;
        ROS_DEBUG_STREAM(*object_ptr);  
        object_ptr++;      

    }
}

std::shared_ptr<VdoSlamInput> RosVdoSlam::pop_vdo_input() {
    queue_mutex.lock();
    std::shared_ptr<VdoSlamInput> input = vdo_input_queue.front();
    vdo_input_queue.pop();
    queue_mutex.unlock();
    return input;
}

void RosVdoSlam::push_vdo_input(std::shared_ptr<VdoSlamInput>& input) {
    queue_mutex.lock();
    vdo_input_queue.push(input);
    queue_mutex.unlock();
}


void RosVdoSlam::vdo_worker() {

    std::unique_ptr<VDO_SLAM::Scene> scene;
    while (ros::ok()) {


        //cam add semaphore here so we waste less CPU time but it seems fine for now
        if (!vdo_input_queue.empty()) {

            std::shared_ptr<VdoSlamInput> input = pop_vdo_input();

            std::unique_ptr<VDO_SLAM::Scene> unique_scene =  slam_system->TrackRGBD(input->raw,input->depth,
                input->flow,
                input->mask,
                input->ground_truth,
                input->object_pose_gt,
                input->time_diff,
                image_trajectory,global_optim_trigger);

            set_scene_labels(unique_scene);
            scene = std::move(unique_scene);
            std::unique_ptr<VDO_SLAM::RosScene> unique_ros_scene = std::unique_ptr<VDO_SLAM::RosScene>(
                    new VDO_SLAM::RosScene(*scene, input->image_time));
            ros_scene = std::move(unique_ros_scene);
            ros_scene_manager.display_scene(ros_scene);
        }
    }

}