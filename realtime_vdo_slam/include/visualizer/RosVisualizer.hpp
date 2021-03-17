#ifndef _ROS_VDO_VISUALIZER
#define _ROS_VDO_VISUALIZER

#include <mutex>
#include <future>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/opencv.hpp>
#include <realtime_vdo_slam/VdoSlamScene.h>

#include "utils/ThreadedQueue.hpp"

namespace VDO_SLAM {

    /**
     * @brief Overlays an image with all information from a Slam Scene message.
     * This includes bounding box, class, tracking ID, pose and velocity
     * 
     * @param image rgb image to overlay
     * @param slam_scene slam scene to details
     * @return cv::Mat overlayed image
     */
    cv::Mat overlay_scene_image(const cv::Mat& image, const realtime_vdo_slam::VdoSlamScenePtr& slam_scene);


    class RosVisualizer {

        public:
            RosVisualizer(ros::NodeHandle& _nh);
            ~RosVisualizer();

            /**
             * @brief Spins the vizualizer asynchronously at a given rate. Upon execution all vdo slam scene messages
             * in the callback queue will be displayed and published. 
             * 
             * @param rate int The rate at which the visualizer will spin. 
             * @return true 
             * @return false 
             */
            bool spin_viz(int rate = 1);

            /**
             * @brief Adds a slam scene to the visualizer queue. Each msg will be published from the queue 
             * at the rate set from spin_viz. 
             * 
             * @param slam_scene realtime_vdo_slam::VdoSlamScenePtr&
             * @return true 
             * @return false 
             */
            bool queue_slam_scene(realtime_vdo_slam::VdoSlamScenePtr& slam_scene);

            /**
             * @brief Subscribes to nav_msgs::Odometry messages that will be used for ground truth. Listenes to the /ros_vdo/odometry_ground_truth_topic
             * defined in the launch file. (see VDO_SLAM::Utils in RosScene.cpp) It will offset the odom gt bt some amount 
             * such that the gt odom starts at [0,0,0][0,0,0,1] even if the ROS information playing does not start centerd at the world 
             * coordinates. This purpose of this is to start the VDO_SLAM odom and the gt odom at the same place so we can compare them visually.
             * 
             * If odom started at the world frame, this would not be necessary as the VDO_SLAM odom starts at [0,0,0][0,0,0,1].
             * Internally, this will add the gt square to the display mat object (or should this be done by a function call?)
             * 
             * @param msg 
             */
            void odom_gt_callback(const nav_msgs::OdometryConstPtr& msg);



        private:

            /**
             * @brief Updates all display and topic messages. Called by spin viz
             * 
             * @param slam_scene realtime_vdo_slam::VdoSlamScenePtr&
             * @param bool if success
             */
            bool update_spin(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene);

            /**
             * @brief Publishes the latest odometry from the VDO slam algorithm. If tf transforms are available,
             * the tf tree will also be updated.
             * 
             *  @param slam_scene realtime_vdo_slam::VdoSlamScenePtr&
             */
            void publish_odom(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene);


            /**
             * @brief Publishes a slam scene as a visualization marker array to be displayed in RVIZ
             * 
             * @param slam_scene realtime_vdo_slam::VdoSlamScenePtr&
             */
            void publish_3D_viz(const realtime_vdo_slam::VdoSlamScenePtr& slam_scene);


            /**
             * @brief Publishes and visualises the birdseye view of the scene. It plots the camera pos as a red square and all tracked
             * 3D objects as coloured dots (currently coloured by classification). If gt odom is present it will also plot this as a green 
             * squares.
             * 
             * @param scene const realtime_vdo_slam::VdoSlamScenePtr&
             */
            void publish_display_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene);

            /**
             * @brief Publishes the slam image with bounding box, class and velocity information draw
             * over the image. Makes a call to overlay_scene_image
             * 
             * @param scene const realtime_vdo_slam::VdoSlamScenePtr
             */
            void publish_bounding_box_mat(const realtime_vdo_slam::VdoSlamScenePtr& scene);


            ros::NodeHandle nh;

            //publish VdoSlamScene msg
            ros::Publisher slam_scene_pub;
            
            //publishes the slam scene as markers for RVIZ
            ros::Publisher slam_scene_3d_pub;

            //publish camera pose from VDO as odom
            ros::Publisher odom_pub;

            image_transport::ImageTransport image_transport;
            image_transport::Publisher bounding_box_pub;

            cv::Mat display; //static and dynamic object track
            image_transport::Publisher object_track_pub;

            //Thread safe queue for VdoSlamScene's
            ThreadsafeQueue<realtime_vdo_slam::VdoSlamScenePtr> slam_scene_queue;


            // subcrsibes to gt odom if exists. Odom gt topic defined in vdo_slam launch file
            std::string odom_gt_topic;
            ros::Subscriber odom_gt_sub;
            std::mutex display_mutex;

            tf2_ros::TransformBroadcaster broadcaster;
            tf2_ros::Buffer tf_buffer;
            tf2_ros::TransformListener listener;

            nav_msgs::Odometry gt_odom;

            const int x_offset = 150;
            const int y_offset = 150;
            const int scale = 6;

            //Info needed for gt odom
            bool is_first;
            float odom_x_offset;
            float odom_y_offset;
            
            //frames for tf tree to publish on
            std::string map_frame;
            std::string odom_frame;
            std::string base_frame;
            


            static int vis_count; //used for marker visualisation
    };

    typedef std::shared_ptr<RosVisualizer> RosVisualizerPtr;
    typedef std::unique_ptr<RosVisualizer> RosVisualizerUniquePtr;
    typedef std::future<bool> RosVizualizerSpinHandler;

    // std::future<bool> data_provider_handle =
    //     std::async(std::launch::async,
    //                &VIO::RosDataProviderInterface::spin,


}


#endif