#include "RealTimeVdoSlam.hpp"
#include "MaskRcnnInterface.hpp"
#include "RosScene.hpp"


#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vdo_slam.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

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
        handle.getParam("/global_optim_trigger", global_optim_trigger);
        ROS_INFO_STREAM("Global Optimization Trigger at frame id: " << global_optim_trigger);

        // first we check if the services exist so we dont start them again
        if  (!ros::service::exists("mask_rcnn_service", true)) {
            ROS_INFO_STREAM("starting mask rcnn service");
            mask_rcnn_interface.start_service();
            ros::service::waitForService("mask_rcnn_service");
            MaskRcnnInterface::set_mask_labels(handle);
        }
        else {
            ROS_INFO_STREAM("Mask Rcnn already active");
        }
        MaskRcnnInterface::set_mask_labels(handle, ros::Duration(2));

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
            ros_scene_manager.update_display_mat(ros_scene);

            cv::imshow("Trajectory", ros_scene_manager.get_display_mat());
            cv::waitKey(1);
        }
    }

}