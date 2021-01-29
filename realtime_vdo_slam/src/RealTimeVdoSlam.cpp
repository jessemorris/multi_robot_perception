#include "RealTimeVdoSlam.hpp"
#include <ros/package.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <vdo_slam.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <memory>

int VDO_SLAM::RosScene::vis_count = 0;

//should also convert unix timestamp to ROS time
//current timetstamp is just time difference and not unix time
VDO_SLAM::RosSceneObject::RosSceneObject(SceneObject& _object, ros::Time& _time, int _uid) :
    SceneObject(_object),
    time(_time),
    uid(_uid) {}

VDO_SLAM::RosScene::RosScene(ros::NodeHandle& _nh, Scene& _object, ros::Time _time) :
    nh(_nh),
    time(_time),
    Scene(_object) {

        visualiser = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/visualization", 20 );
        odom_pub = nh.advertise<nav_msgs::Odometry>("vdoslam/odom", 20);
        nh.getParam("/frame_id", child_frame_id);
        ROS_INFO_STREAM("Setting child frame it: " << child_frame_id);
        ROS_INFO_STREAM("Camera pos: " << camera_pos_translation);
        ROS_INFO_STREAM("Camera rot: " << camera_pos_rotation);

        //convert them all into RosSceneObjects
        int id = 0;
        for(int i = 0; i < scene_objects.size(); i++) {
            RosSceneObject ros_scene_object(scene_objects[i], time, id);
            scene_objects[i] = ros_scene_object;
            id++;
        }

        geometry_msgs::TransformStamped transform_stamped;

        //update tf pose for ROS
        transform_stamped.header.stamp = time;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = child_frame_id;
        transform_stamped.transform.translation.x = camera_pos_translation.x;
        transform_stamped.transform.translation.y = camera_pos_translation.y;
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

        broadcaster.sendTransform(transform_stamped);
        ROS_INFO_STREAM("Published transform");

        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = child_frame_id;
        odom.pose.pose.position.x = camera_pos_translation.x;
        odom.pose.pose.position.y = camera_pos_translation.y;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        odom.twist.twist.linear.x = camera_vel_translation.x;
        odom.twist.twist.linear.y = camera_vel_translation.y;
        odom.twist.twist.linear.z = camera_vel_translation.z;

        //TODO: angular velocity

        odom_pub.publish(odom);


    }


void VDO_SLAM::RosScene::display_scene() {
    visualization_msgs::MarkerArray marker_array;
    for (SceneObject& scene_object: scene_objects) {
        ROS_INFO_STREAM(scene_object);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = time;
        marker.ns = "vdoslam";
        marker.id = scene_object.tracking_id;
        // marker.type = visualization_msgs::Marker::SPHERE;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = scene_object.pose.x;
        marker.pose.position.y = scene_object.pose.y;
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
        vis_count++;
    }

    visualiser.publish(marker_array);
}


RealTimeVdoSLAM::RealTimeVdoSLAM(ros::NodeHandle& n) :
        handler(n),
        sceneflow(n),
        mask_rcnn_interface(n),
        mono_depth(n),
        image_transport(n),
        is_first(true),
        scene_flow_success(false),
        mask_rcnn_success(false),
        mono_depth_success(false)
{
    //set up config
    //TODO: param not working?
    handler.param<std::string>("/topic_prefix", topic_prefix, "/gmsl/");
    handler.param<std::string>("/camera_suffix", camera_suffix, "/image_color");
    handler.param<std::string>("/info_msg_suffix", info_msg_suffix, "/camera_info");

    handler.param<std::string>("/camera_selection", camera_selection, "A0");


    handler.param<bool>("/apply_undistortion", undistord_images, false);
    handler.param<bool>("/run_mask_rcnn", run_mask_rcnn, false);
    handler.param<bool>("/run_flow_net", run_scene_flow, false);
    handler.param<bool>("/run_mono_depth", run_mono_depth, false);

    //after how many images full batch oprtimisation should be run
    handler.param<int>("/global_optim_trigger", global_optim_trigger, 10);
    ROS_INFO_STREAM("global optim trigger " << global_optim_trigger);

    if(run_mask_rcnn) {
        //first we check if the services exist so we dont start them again
        if  (!ros::service::exists("mask_rcnn_service", true)) {
            ROS_INFO_STREAM("starting mask rcnn service");
            mask_rcnn_interface.start_service();
            ros::service::waitForService("mask_rcnn_service");
            ros::service::waitForService("mask_rcnn_label_list");
        }
        else {
            ROS_INFO_STREAM("Mask Rcnn already active");
        }
        mask_rcnn_interface.set_mask_labels();
    }
    if(run_scene_flow) {
        if  (!ros::service::exists("flow_net_service", true)) { 
            ROS_INFO_STREAM("starting flow net service");
            sceneflow.start_service();
            ros::service::waitForService("flow_net_service");
        }
        else {
            ROS_INFO_STREAM("Flow Net already active");
        }
    }

    if(run_mono_depth) {
        if  (!ros::service::exists("mono_depth_service", true)) { 
            ROS_INFO_STREAM("starting mono_depth service");
            mono_depth.start_service();
            ros::service::waitForService("mono_depth_service");
        }
        else {
            ROS_INFO_STREAM("MonoDepth already active");
        }
    }

    ROS_INFO_STREAM("camera selection " << camera_selection);

    // gmsl/<>/image_colour
    // output_video_topic = topic_prefix + camera_selection + camera_suffix;
    output_video_topic = "/camera/raw_image";
    camea_info_topic = topic_prefix + camera_selection + info_msg_suffix;

    ROS_INFO_STREAM("video topic " << output_video_topic);
    ROS_INFO_STREAM("camera info topic " << camea_info_topic);

    camera_information.topic = output_video_topic;
    // ROS_INFO_STREAM("Waiting for camera info msg");
    // sensor_msgs::CameraInfoConstPtr camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camea_info_topic);
    // camera_information.camera_info = *camera_info;
    // ROS_INFO_STREAM("Got camera info msg");


    image_subscriber = image_transport.subscribe(output_video_topic, 10,
                                               &RealTimeVdoSLAM::image_callback, this);

    maskrcnn_results = image_transport.advertise("vdoslam/results/maskrcnn", 10);
    flownet_results = image_transport.advertise("vdoslam/results/flownet", 10);
    monodepth_results = image_transport.advertise("vdoslam/results/monodepth", 10);


    std::string path = ros::package::getPath("realtime_vdo_slam");
    std::string vdo_slam_config_path = path + "/config/vdo_config.yaml";
    // VDO_SLAM::System SLAM(vdo_slam_config_path,VDO_SLAM::System::RGBD);
    slam_system = std::make_unique< VDO_SLAM::System>(vdo_slam_config_path,VDO_SLAM::System::RGBD);
    image_trajectory = cv::Mat::zeros(800, 600, CV_8UC3);

    vdo_worker_thread = std::thread(&RealTimeVdoSLAM::vdo_worker, this);
}

RealTimeVdoSLAM::~RealTimeVdoSLAM() {
    if (vdo_worker_thread.joinable()) {
        vdo_worker_thread.join();
    }

}

void RealTimeVdoSLAM::image_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat distored = cv_ptr->image;
    // cv::cvtColor(distored, distored, CV_RGB2BGR);
    cv::Mat image = distored;


    // if (undistord_images) {
    //     cv::undistort(distored, image, camera_information.intrinsic, camera_information.distortion);
    // }
    // else {
    //     image = distored;
    // }
    // cv::Mat flow_matrix, segmentation_mask;
    std::vector<std::string> mask_rcnn_labels;
    std::vector<int> mask_rcnn_label_indexs;


    if (is_first) {
        previous_image = image;
        // previous_time = msg->header.stamp;
        previous_time = ros::Time::now();
        is_first = false;
        return;
    }
    else {
        cv::Mat current_image = image;
        // current_time = msg->header.stamp;
        current_time = ros::Time::now();
        if (run_scene_flow) {
            scene_flow_success = sceneflow.analyse_image(current_image, previous_image, scene_flow_mat);

            if (scene_flow_success) {
                //#TODO: cannot fisplay until convert from scene flow to rgb
                // std_msgs::Header header = std_msgs::Header();

               
                // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "rgb8", scene_flow_mat).toImageMsg();
                // flownet_results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse scene flow images");
            }
        }

        if (run_mask_rcnn) {
            mask_rcnn_success = mask_rcnn_interface.analyse_image(current_image, mask_rcnn_mat, mask_rcnn_labels, mask_rcnn_label_indexs);

            if (mask_rcnn_success) {
                std_msgs::Header header = std_msgs::Header();

                // //TODO: proper headers
                // header.frame_id = "base_link";
                // header.stamp = ros::Time::now();
                // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono8", mask_rcnn_mat).toImageMsg();
                // maskrcnn_results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mask rcnn images");
            }
        }

        if (run_mono_depth) {
            mono_depth_success = mono_depth.analyse_image(current_image, mono_depth_mat);

            if (mono_depth_success) {
                std_msgs::Header header = std_msgs::Header();

                // sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "mono16", mono_depth_mat).toImageMsg();
                // monodepth_results.publish(img_msg);
            }
            else {
                ROS_WARN_STREAM("Could not analyse mono depthimages");
            }
        }


        previous_image = current_image;


        //run slam algorithm
        //not we have no ground truth
        //times are just relative to each other so we can just record rostim between each callback
        if (scene_flow_success && mask_rcnn_success && mono_depth_success) {
            ros::Duration diff = current_time - previous_time;
            //time should be in n seconds or seconds (or else?)
            double time_difference = diff.toNSec();

            std::shared_ptr<VdoSlamInput> input = std::make_shared<VdoSlamInput>(image,scene_flow_mat,
                    mono_depth_mat, mask_rcnn_mat, time_difference, current_time);

            //add the input to the thread queue so we can deal with it later
            push_vdo_input(input);
        }

        previous_time = current_time;


    }



}

void RealTimeVdoSLAM::set_scene_labels(std::unique_ptr<VDO_SLAM::Scene>& scene) {
    int size = scene->scene_objects_size();
    VDO_SLAM::SceneObject* object_ptr = scene->get_scene_objects_ptr();
    ROS_DEBUG_STREAM("Updating scene labels (size " << size << ")");
    for(int i = 0; i < size; i++) {
        std::string label = mask_rcnn_interface.request_label(object_ptr->label_index);
        object_ptr->label = label;
        ROS_DEBUG_STREAM(*object_ptr);  
        object_ptr++;      

    }
}

std::shared_ptr<VdoSlamInput> RealTimeVdoSLAM::pop_vdo_input() {
    queue_mutex.lock();
    std::shared_ptr<VdoSlamInput> input = vdo_input_queue.front();
    vdo_input_queue.pop();
    queue_mutex.unlock();
    return input;
}
void RealTimeVdoSLAM::push_vdo_input(std::shared_ptr<VdoSlamInput>& input) {
    queue_mutex.lock();
    vdo_input_queue.push(input);
    queue_mutex.unlock();
}


void RealTimeVdoSLAM::vdo_worker() {

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
            std::unique_ptr<VDO_SLAM::RosScene> unique_ros_scene = std::unique_ptr<VDO_SLAM::RosScene>(new VDO_SLAM::RosScene(handler, *scene, input->image_time));
            ros_scene = std::move(unique_ros_scene);
            ros_scene->display_scene();
        }
    }

}