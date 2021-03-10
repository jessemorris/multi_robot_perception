#include "RosVdoSlam.hpp"
#include "RosScene.hpp"
#include "RosSceneManager.hpp"
#include "VdoSlamInput.hpp"
#include "CameraInformation.hpp"
#include "utils/RosUtils.hpp"


#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vdo_slam/vdo_slam.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

using namespace VDO_SLAM;

RosVdoSlam::RosVdoSlam(ros::NodeHandle& n) :
        handle(n),
        ros_scene_manager(handle),
        mask_rcnn_interface(n),
        raw_img(handle,"/camera/rgb/image_raw", 100),
        mask_img(handle,"/camera/mask/image_raw", 100),
        flow_img(handle,"/camera/flow/image_raw", 100),
        depth_img(handle,"/camera/depth/image_raw", 100),
        sync(raw_img, mask_img, flow_img, depth_img, 100)

    {
        //Getting Frame ID's
        handle.getParam("/ros_vdoslam/map_frame_id", map_frame_id);
        handle.getParam("/ros_vdoslam/odom_frame_id", odom_frame_id);
        handle.getParam("/ros_vdoslam/base_link_frame_id", base_link_frame_id);

        nav_msgs::Odometry odom;
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        //setting map frame and odom frame to be the same
        VDO_SLAM::utils::publish_static_tf(odom, map_frame_id, odom_frame_id);

        //setting base link starting point to be the same as odom
        VDO_SLAM::utils::publish_static_tf(odom, odom_frame_id, base_link_frame_id);


        handle.getParam("/ros_vdoslam/optimization_trigger_frame", global_optim_trigger);
        ROS_INFO_STREAM("Global Optimization Trigger at frame id: " << global_optim_trigger);

        // first we check if the services exist so we dont start them again
        mask_rcnn_interface.start_service();
        mask_rcnn::MaskRcnnInterface::set_mask_labels(handle, ros::Duration(2));

        //TODO: should get proper previous time
        previous_time = ros::Time::now();
        image_trajectory = cv::Mat::zeros(800, 600, CV_8UC3);


        // slam_system = std::make_unique< VDO_SLAM::System>(vdo_slam_config_path,VDO_SLAM::eSensor::MONOCULAR);
        // slam_system = std::move(construct_slam_system(handle));
        slam_system = construct_slam_system(handle);

        sync.registerCallback(boost::bind(&RosVdoSlam::vdo_input_callback, this, _1, _2, _3, _4));
        vdo_worker_thread = std::thread(&RosVdoSlam::vdo_worker, this);




    }

RosVdoSlam::~RosVdoSlam() {
    if (vdo_worker_thread.joinable()) {
        vdo_worker_thread.join();
    }
}

std::shared_ptr<VDO_SLAM::System> RosVdoSlam::construct_slam_system(ros::NodeHandle& nh) {
    //check first where to load the params from - calibration file or launch file
    bool use_calibration_file;
    nh.getParam("/ros_vdoslam/use_calibration_file", use_calibration_file);

    //load from calibration file
    if(use_calibration_file) {
        int sensor_mode;
        nh.getParam("/ros_vdoslam/sensor_mode", sensor_mode);

        VDO_SLAM::eSensor sensor;

        if (sensor_mode == 0) {
            sensor = VDO_SLAM::eSensor::MONOCULAR;
        }
        else if (sensor_mode == 1) {
            sensor = VDO_SLAM::eSensor::STEREO;
        }
        else if (sensor_mode == 2) {
            sensor = VDO_SLAM::eSensor::RGBD;
        }

        std::string calibration_file;
        nh.getParam("/ros_vdoslam/calibration_file", calibration_file);

        std::string path = ros::package::getPath("realtime_vdo_slam");
        std::string vdo_slam_config_path = path + "/config/" + calibration_file;


        return std::make_shared<VDO_SLAM::System>(vdo_slam_config_path, sensor);
    }
    //load from params in launch
    else {
        VDO_SLAM::VdoParams params;
        //check if we should get instrincs from camera info topic
        bool use_camera_info_topic;
        nh.getParam("/ros_vdoslam/use_camera_info_topic", use_camera_info_topic);

        //wait for topic defined in configuration file
        if (use_camera_info_topic) {
            std::string camera_info_topic = "/camera/camera_info";
            // nh.getParam("/ros_vdoslam/input_camera_info_topic", camera_info_topic);
            ROS_INFO_STREAM("Waiting for camera info topic: " << camera_info_topic);
            sensor_msgs::CameraInfoConstPtr info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic);

            VDO_SLAM::CameraInformation cam_info(info_ptr);
            
            bool apply_undistortion;
            nh.getParam("/ros_vdoslam/apply_undistortion", apply_undistortion);

            //if true -> we use the modified camera matrix P
            if(apply_undistortion) {
                ROS_INFO_STREAM(cam_info.modified_camera_matrix.size());
                ROS_INFO_STREAM(cam_info.modified_camera_matrix);
                //should be 3x3
                params.fx = cam_info.modified_camera_matrix.at<double>(0,0);
                params.cx = cam_info.modified_camera_matrix.at<double>(0,2);
                params.fy = cam_info.modified_camera_matrix.at<double>(1,1);
                params.cy = cam_info.modified_camera_matrix.at<double>(1,2);
            }
            //else use the original matrix K
            else {
                params.fx = cam_info.camera_matrix.at<double>(0,0);
                params.cx = cam_info.camera_matrix.at<double>(0,2);
                params.fy = cam_info.camera_matrix.at<double>(1,1);
                params.cy = cam_info.camera_matrix.at<double>(1,2);
            }
        }
        //load camera params from launch file
        else {
            
            nh.getParam("/ros_vdoslam/fx", params.fx);
            nh.getParam("/ros_vdoslam/fy", params.fy);
            nh.getParam("/ros_vdoslam/cx", params.cx);
            nh.getParam("/ros_vdoslam/cy", params.cy);
            
        }

        //for now just set dist params to 0
        params.k1 = 0;
        params.k2 = 0;
        params.p1 = 0;
        params.p2 = 0;
        params.p3 = 0;

        //load rest of params from file
        nh.getParam("/ros_vdoslam/width", params.width);
        nh.getParam("/ros_vdoslam/height", params.height);

        nh.getParam("/ros_vdoslam/fps", params.fps);
        nh.getParam("/ros_vdoslam/bf", params.bf);

        nh.getParam("/ros_vdoslam/RGB", params.RGB);
        nh.getParam("/ros_vdoslam/data_code", params.data_code);

        int sensor_mode;
        nh.getParam("/ros_vdoslam/sensor_type", sensor_mode);

        if (sensor_mode == 0) {
            params.sensor_type = VDO_SLAM::eSensor::MONOCULAR;
        }
        else if (sensor_mode == 1) {
            params.sensor_type = VDO_SLAM::eSensor::STEREO;
        }
        else if (sensor_mode == 2) {
            params.sensor_type = VDO_SLAM::eSensor::RGBD;
        }


        nh.getParam("/ros_vdoslam/depth_map_factor", params.depth_map_factor);

        nh.getParam("/ros_vdoslam/thdepth_bg", params.thdepth_bg);
        nh.getParam("/ros_vdoslam/thdepth_obj", params.thdepth_obj);

        nh.getParam("/ros_vdoslam/max_track_points_bg", params.max_track_points_bg);
        nh.getParam("/ros_vdoslam/max_track_points_obj", params.max_track_points_obj);

        nh.getParam("/ros_vdoslam/sf_mg_thresh", params.sf_mg_thresh);
        nh.getParam("/ros_vdoslam/sf_ds_thresh", params.sf_ds_thresh);

        nh.getParam("/ros_vdoslam/window_size", params.window_size);
        nh.getParam("/ros_vdoslam/overlap_size", params.overlap_size);

        nh.getParam("/ros_vdoslam/use_sample_feature", params.use_sample_feature);


        nh.getParam("/ros_vdoslam/n_features", params.n_features);

        nh.getParam("/ros_vdoslam/scale_factor", params.scale_factor);

        nh.getParam("/ros_vdoslam/n_levels", params.n_levels);

        nh.getParam("/ros_vdoslam/ini_th_fast", params.ini_th_fast);
        nh.getParam("/ros_vdoslam/min_th_fast", params.min_th_fast);

        return std::make_shared<VDO_SLAM::System>(params);

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
        std::string label = mask_rcnn::MaskRcnnInterface::request_label(object_ptr->label_index);
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
                    new VDO_SLAM::RosScene(*scene, input->image_time, odom_frame_id, base_link_frame_id));
            ros_scene = std::move(unique_ros_scene);
            ros_scene_manager.display_scene(ros_scene);
            ros_scene_manager.update_display_mat(ros_scene);

            cv::imshow("Trajectory", ros_scene_manager.get_display_mat());
            cv::waitKey(1);
        }
    }

}