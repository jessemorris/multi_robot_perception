#include "visualizer/RosVisualizer.hpp"
#include "VdoSlamMsgInterface.hpp"

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>
#include <realtime_vdo_slam/VdoSceneObject.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <string>

#include "utils/RosUtils.hpp"
#include "VdoSlamMsgInterface.hpp"


using namespace VDO_SLAM;


cv::Mat VDO_SLAM::overlay_scene_image(const cv::Mat& image, const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
    cv::Mat overlayed;
    image.copyTo(overlayed);

    if(slam_scene != nullptr) {
        std::vector<realtime_vdo_slam::VdoSceneObject> objects = slam_scene->objects;

        for(realtime_vdo_slam::VdoSceneObject& object : objects) {
            //draw bounding box
            cv::rectangle(overlayed, cv::Rect2d(object.bounding_box.center.x, object.bounding_box.center.y,
                object.bounding_box.size_x, object.bounding_box.size_y), 
                cv::Scalar(0, 255, 0), 2);


            //add info
            char text[200];
            sprintf(text, "%s, [%d] %02fkm/h", object.label.c_str(), object.tracking_id, object.twist.linear.x);
            cv::putText(overlayed, text, cv::Point(object.bounding_box.center.x, object.bounding_box.center.y), 
                cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

        }
    }

    return overlayed;
}

namespace VDO_SLAM {


    RosVisualizer::RosVisualizer(VisualizerParamsPtr& params_)
        :   Visualizer2D(params_),
            nh("ros_visualizer"),
            image_transport(nh),
            listener(tf_buffer) {

            nh.getParam("/vdo_pipeline/visualizer/display_window", display_window);

            //create mems for async callback queues
            vdo_scene_queue_ptr = std::make_shared<ros::CallbackQueue>();
            publish_queue_ptr = std::make_shared<ros::CallbackQueue>();

            async_manager = std::make_shared<RosAsyncManager>(publish_queue_ptr);


            //create async subscriber to our ros scene
            static constexpr size_t kMaxSceneQueueSize = 1000u;
            ros::SubscribeOptions vdo_slam_subscriber_options =
                ros::SubscribeOptions::create<realtime_vdo_slam::VdoSlamScene>(
                    "/vdoslam/output/scene",
                    kMaxSceneQueueSize,
                    boost::bind(&RosVisualizer::slam_scene_callback, this, _1),
                    ros::VoidPtr(),
                    vdo_scene_queue_ptr.get());

            vdo_slam_subscriber_options.transport_hints.tcpNoDelay(true);
            slam_scene_sub = nh.subscribe(vdo_slam_subscriber_options);


            ros::SubscribeOptions vdo_slam_map_subscriber_options =
                ros::SubscribeOptions::create<realtime_vdo_slam::VdoSlamMap>(
                    "/vdoslam/output/map",
                    kMaxSceneQueueSize,
                    boost::bind(&RosVisualizer::reconstruct_scenes_callback, this, _1),
                    ros::VoidPtr(),
                    vdo_scene_queue_ptr.get());

            vdo_slam_map_subscriber_options.transport_hints.tcpNoDelay(true);
            slam_map_sub = nh.subscribe(vdo_slam_map_subscriber_options);

            //create the actual async spinner to listen to the vdoslam/output/scene topic
            static constexpr size_t kSceneSpinnerThreads = 2u;
            async_spinner_scene =
                std::make_unique<ros::AsyncSpinner>(kSceneSpinnerThreads, vdo_scene_queue_ptr.get());

            //create all publishers that will use publish_queue_ptr (rather than the global queue)
            //they will eventually be attached to async_spinner_publish
            async_manager->create<visualization_msgs::MarkerArray>("/vdoslam/output/3dscene",
                    slam_scene_3d_pub, nh);

            async_manager->create<nav_msgs::Odometry>("/vdoslam/output/odom",
                    odom_pub, nh);

            async_manager->create<sensor_msgs::Image>("/vdoslam/output/bounding_box_image",
                bounding_box_pub, image_transport);

            async_manager->create<sensor_msgs::Image>("/vdoslam/output/object_point_image",
                object_track_pub, image_transport);

            static constexpr size_t kPublishSpinnerThreads = 2u;
            //create the ros spinner that is responsible for publishing all visualization messages
            async_spinner_publish =
                std::make_unique<ros::AsyncSpinner>(kPublishSpinnerThreads, publish_queue_ptr.get());


            nh.getParam("/vdo_pipeline/visualizer/odometry_ground_truth_topic", odom_gt_topic);
            nh.getParam("/vdo_pipeline/visualizer/gps_topic", gps_topic);


            //Getting Frame ID's
            nh.getParam("/vdo_pipeline/map_frame_id", map_frame);
            nh.getParam("/vdo_pipeline/odom_frame_id", odom_frame);
            nh.getParam("/vdo_pipeline/base_link_frame_id", base_frame);

            if(gt_odom_in_use()) {
                odom_gt_sub = nh.subscribe<nav_msgs::Odometry>(odom_gt_topic, 100, &RosVisualizer::odom_gt_callback, this);
            }

            if(gps_in_use()) {
                gps_sub = nh.subscribe<sensor_msgs::NavSatFix>(gps_topic, 100, &RosVisualizer::gps_callback, this);
            }


            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = 0;
            odom.pose.pose.position.y = 0;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;

            //setting map frame and odom frame to be the same
            VDO_SLAM::utils::publish_static_tf(odom, map_frame, odom_frame);

            //setting base link starting point to be the same as odom
            VDO_SLAM::utils::publish_static_tf(odom, odom_frame, base_frame);

            display = cv::Mat::zeros(800, 800, CV_8UC3);

            ROS_INFO_STREAM("Created VDO_SLAM visualizer");

    }

    RosVisualizer::~RosVisualizer() {
        slam_scene_queue.shutdown();
        ROS_INFO_STREAM("Shutting down visualizer with " << slam_scene_queue.size());

        if(scene_spinner_started) {
            async_spinner_scene->stop();
        }
        if (publish_spinner_started) {
            async_spinner_publish->stop();
        }
    }

    void RosVisualizer::connect_handler(RosVizualizerSpinHandler& handler) {
        handler = std::async(std::launch::async,
                   &VDO_SLAM::RosVisualizer::spin_viz,
                   this);
    }

    bool RosVisualizer::spin_viz() {

        async_spinner_scene->start();
        scene_spinner_started = true;
        ROS_INFO_STREAM("Async Scene spinner started");

        async_spinner_publish->start();
        publish_spinner_started = true;
        ROS_INFO_STREAM("Async Publish spinner started");

        return true;

    }

    void RosVisualizer::slam_scene_callback(const realtime_vdo_slam::VdoSlamSceneConstPtr& slam_scene) {
        realtime_vdo_slam::VdoSlamSceneConstPtr slam_scene_ptr = slam_scene;
        // boost::shared_ptr<realtime_vdo_slam::VdoSlamScene> slam_scene_ptr(boost::const_pointer_cast<realtime_vdo_slam::VdoSlamScene>(slam_scene));
        // update_spin(slam_scene_ptr);
        SlamScenePtr scene = Scene::create<realtime_vdo_slam::VdoSlamSceneConstPtr>(slam_scene_ptr);
        VisualizerOutputUniquePtr viz_output =  spinOnce(scene);
    }

    void RosVisualizer::reconstruct_scenes_callback(const realtime_vdo_slam::VdoSlamMapConstPtr& map) {
        // ROS_INFO_STREAM("Recieved slam map updated");

        // //reset display
        // display = cv::Mat::zeros(800, 800, CV_8UC3);
        // for(const realtime_vdo_slam::VdoSlamScene& scene: map->scenes) {
        //     boost::shared_ptr<realtime_vdo_slam::VdoSlamScene> slam_scene_ptr = boost::make_shared<realtime_vdo_slam::VdoSlamScene>(scene);
        //     update_spin(slam_scene_ptr);
        // }
    }


    void RosVisualizer::odom_gt_callback(const nav_msgs::OdometryConstPtr& msg) {
        if (is_first_odom) {
            odom_x_offset = msg->pose.pose.position.x;
            odom_y_offset = msg->pose.pose.position.y;
            is_first_odom = false;
        }
        gt_odom.pose.pose.position.x = odom_x_offset - msg->pose.pose.position.x ;
        gt_odom.pose.pose.position.y = odom_y_offset - msg->pose.pose.position.y;
        gt_odom.pose.pose.position.z = 0;

        gt_odom.pose.pose.orientation = msg->pose.pose.orientation;
    }

    void RosVisualizer::gps_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
        if (is_first_gps) {
            double easting, northing;
            std::string zone;

            double lat = msg->latitude;
            double longitude = msg->longitude;

            utils::LLtoUTM(lat, longitude, northing, easting, zone);
            ROS_INFO_STREAM("Updating map topic with easting: " << easting << " northing: " << northing);

            nav_msgs::Odometry odom;
            odom.pose.pose.position.x = easting;
            odom.pose.pose.position.y = northing;
            odom.pose.pose.position.z = 0;

            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;
            odom.pose.pose.orientation.w = 1;
            is_first_gps = false;

            VDO_SLAM::utils::publish_static_tf(odom, map_frame, odom_frame);

        }
    }

    ros::Publisher RosVisualizer::create_viz_pub(ros::NodeHandle& nh) {
        return nh.advertise<realtime_vdo_slam::VdoSlamScene>("/vdoslam/output/scene", 10);
    }

    ros::Publisher RosVisualizer::create_map_update_pub(ros::NodeHandle& nh) {
        return nh.advertise<realtime_vdo_slam::VdoSlamMap>("/vdoslam/output/map", 10);
    }

    // bool RosVisualizer::update_spin(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
    //     publish_odom(slam_scene);
    //     publish_3D_viz(slam_scene);
    //     publish_display_mat(slam_scene);
    //     publish_bounding_box_mat(slam_scene);
    // }


    void RosVisualizer::publish_odom(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
        nav_msgs::Odometry odom;
        utils::geometry_to_odom(slam_scene->camera_pose, slam_scene->camera_twist, 
            slam_scene->header.stamp, odom_frame, base_frame, odom);

        odom_pub.publish(odom);

        utils::publish_static_tf(odom, odom_frame, base_frame);

    }

    void RosVisualizer::publish_3D_viz(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene) {
        visualization_msgs::MarkerArray marker_array;
        int i = 0;
        for (realtime_vdo_slam::VdoSceneObject& scene_object : slam_scene->objects) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = odom_frame;
            marker.header.stamp = slam_scene->header.stamp;
            marker.text = scene_object.label;
            marker.ns = "vdoslam";
            marker.id = i;
            // marker.type = visualization_msgs::Marker::SPHERE;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = scene_object.pose.position.x;
            marker.pose.position.y = scene_object.pose.position.y;
            marker.pose.position.z = 0;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 2;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(2.0);
            // marker.text = scene_object.label;
            marker.text = scene_object.tracking_id;
            marker_array.markers.push_back(marker);
            i++;
        }

        slam_scene_3d_pub.publish(marker_array);

    }

    void RosVisualizer::publish_display_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene) {
        // update_display_mat(scene);

        if(display_window) {
            cv::imshow("Object points and camera trajectory", display);
            cv::waitKey(1);
        }

        sensor_msgs::Image img_msg;
        utils::mat_to_image_msg(img_msg, display, sensor_msgs::image_encodings::BGR8, scene->header);
        object_track_pub.publish(img_msg);
    }

    // void RosVisualizer::update_display_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene) {
    //     display_mutex.lock();
    //     geometry_msgs::Pose pose = scene->camera_pose;

    //     double x = pose.position.x;
    //     double y = pose.position.y;
    //     double z = pose.position.z;

    //     int x_display =  static_cast<int>(x*scale) + x_offset;
    //     int y_display =  static_cast<int>(y*scale) + y_offset;

    //     //add odom to cv mat
    //     cv::rectangle(display, cv::Point(x_display, y_display), cv::Point(x_display+10, y_display+10), cv::Scalar(0,0,255),1);
    //     cv::rectangle(display, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), CV_FILLED);
    //     cv::putText(display, "Camera Trajectory (RED SQUARE)", cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    //     char text[100];
    //     sprintf(text, "x = %02fm y = %02fm z = %02fm", x, y, z);
    //     cv::putText(display, text, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

    //     if (gt_odom_in_use()) {
    //         // ROS_INFO_STREAM(gt_odom);
    //         x = gt_odom.pose.pose.position.x;
    //         y = gt_odom.pose.pose.position.y;
    //         z = gt_odom.pose.pose.position.z;
    //         //here we update the odom repub to the display mat
    //         //we use 10 for scale    
    //         x_display = static_cast<int>(x*scale) + x_offset;
    //         y_display = static_cast<int>(y*scale) + y_offset;
    //         //add odom to cv mat
    //         cv::rectangle(display, cv::Point(y_display, x_display), cv::Point(y_display+10, x_display+10), cv::Scalar(0,255,0),1);
    //         cv::rectangle(display, cv::Point(10, 100), cv::Point(550, 130), CV_RGB(0,0,0), CV_FILLED);
    //         cv::putText(display, "Camera GT Trajectory (GREEN SQUARE)", cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    //         char text1[100];
    //         sprintf(text1, "x = %.2f y = %.2f z = %.2f", x, y, z);
    //         cv::putText(display, text1, cv::Point(10, 120), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);
    //     }

    //     //draw each object
    //     std::vector<realtime_vdo_slam::VdoSceneObject> scene_objects = scene->objects;
    //     for(realtime_vdo_slam::VdoSceneObject& scene_object : scene_objects) {
    //         double x = scene_object.pose.position.x; 
    //         double y = scene_object.pose.position.y;

    //         ros::Time t = scene->header.stamp;


    //         int x_display =  static_cast<int>(x*scale) + x_offset;
    //         int y_display =  static_cast<int>(y*scale) + y_offset;
    //         int track = scene_object.tracking_id;

    //         HashableColor color = color_manager.get_colour_for_tracking_id(track);
    //         cv::circle(display, cv::Point(x_display, y_display), 2, color, 5);

    //         // //hack for viz so colours repeat
    //         // if (track > 25) {
    //         //     track = track % 25;
    //         // }

    //         // // TODO: use colour conversion function
    //         // switch (track) {

    //         //     case 1:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // orange
    //         //         break;
    //         //     case 2:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // green
    //         //         break;
    //         //     case 3:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0, 255, 0), 5); // yellow
    //         //         break;
    //         //     case 4:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // pink
    //         //         break;
    //         //     case 5:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,0), 5); // cyan (yellow green 47,255,173)
    //         //         break;
    //         //     case 6:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // purple
    //         //         break;
    //         //     case 7:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,255), 5);  // white
    //         //         break;
    //         //     case 8:
    //         //         cv::circle(display,cv::Point(x_display, y_display), 2, CV_RGB(196,228,255), 5); // bisque
    //         //         break;
    //         //     case 9:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(180, 105, 255), 5);  // blue
    //         //         break;
    //         //     case 10:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(42,42,165), 5);  // brown
    //         //         break;
    //         //     case 11:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(35, 142, 107), 5);
    //         //         break;
    //         //     case 12:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(45, 82, 160), 5);
    //         //         break;
    //         //     case 13:
    //         //         cv::circle(display,  cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // red
    //         //         break;
    //         //     case 14:
    //         //         cv::circle(display,cv::Point(x_display, y_display), 2, CV_RGB(255, 165, 0), 5);
    //         //         break;
    //         //     case 15:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,255,0), 5);
    //         //         break;
    //         //     case 16:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,0), 5);
    //         //         break;
    //         //     case 17:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,192,203), 5);
    //         //         break;
    //         //     case 18:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,255,255), 5);
    //         //         break;
    //         //     case 19:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5);
    //         //         break;
    //         //     case 20:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,255), 5);
    //         //         break;
    //         //     case 21:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,228,196), 5);
    //         //         break;
    //         //     case 22:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(180, 105, 255), 5);
    //         //         break;
    //         //     case 23:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(165,42,42), 5);
    //         //         break;
    //         //     case 24:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(35, 142, 107), 5);
    //         //         break;
    //         //     case 25:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(45, 82, 160), 5);
    //         //         break;
    //         //     case 41:
    //         //         cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(60, 20, 220), 5);
    //         //         break;
    //         //     default:
    //         //         break;
    //         //     // default:
    //         //     //     ROS_WARN_STREAM("No case for this segmentaion index yet");
    //         //     //     break;
    //         // }

    //     }

    //     display_mutex.unlock();



    // }

    void RosVisualizer::publish_bounding_box_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene) {
        // cv::Mat raw_img;
        // utils::image_msg_to_mat(raw_img, scene->original_frame, sensor_msgs::image_encodings::RGB8);
        // cv::Mat viz = overlay_scene_image(raw_img, scene);

        // if(display_window) {
        //     cv::imshow("Bounding Box and Tracking IDs", viz);
        //     cv::waitKey(1);
        // }

        // sensor_msgs::Image img_msg;

        // utils::mat_to_image_msg(img_msg, viz, sensor_msgs::image_encodings::RGB8, scene->header);

        // bounding_box_pub.publish(img_msg);
        // cv::imshow("BB and vel", viz);
        // cv::waitKey(1);

    }







}