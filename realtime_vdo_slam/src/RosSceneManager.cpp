#include "RosSceneManager.hpp"
#include "utils/RosUtils.hpp"



int VDO_SLAM::RosSceneManager::vis_count = 0;


VDO_SLAM::RosSceneManager::RosSceneManager(ros::NodeHandle& _nh) :
        nh(_nh),
        listener(tf_buffer)
    {
        visualiser = nh.advertise<visualization_msgs::MarkerArray>("vdoslam/visualization", 20 );
        odom_pub = nh.advertise<nav_msgs::Odometry>("vdoslam/odom", 20);
        odom_repub_sub = nh.subscribe<nav_msgs::Odometry>("/odom_repub", 100, &RosSceneManager::odom_repub_callback, this);
        // nh.getParam("/frame_id", child_frame_id);

        display = cv::Mat::zeros(800, 800, CV_8UC3);
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

void VDO_SLAM::RosSceneManager::odom_repub_callback(const nav_msgs::OdometryConstPtr& msg) {
    gt_odom = *msg;
}

cv::Mat& VDO_SLAM::RosSceneManager::get_display_mat() {
    return display;
}



void VDO_SLAM::RosSceneManager::update_display_mat(std::unique_ptr<VDO_SLAM::RosScene>& scene) {

    const nav_msgs::Odometry odom = scene->odom_msg();

    //camera is offset from the base link so try and orietnate 
    geometry_msgs::TransformStamped transform_stamped;
    geometry_msgs::Pose pose;
    pose.position = odom.pose.pose.position;
    pose.orientation = odom.pose.pose.orientation;

    geometry_msgs::Pose transformed_pose;
    

    double x = odom.pose.pose.position.x;
    double y = odom.pose.pose.position.y;
    double z = odom.pose.pose.position.z;


    //800 is height of cv mat and we want to draw from bottom left
    int x_display =  static_cast<int>(x*scale) + x_offset;
    int y_display =  static_cast<int>(y*scale) + y_offset;
    ROS_INFO_STREAM("x display " << x_display << " y display " << y_display);
    //add odom to cv mat
    display_mutex.lock();
    cv::rectangle(display, cv::Point(x_display, y_display), cv::Point(x_display+10, y_display+10), cv::Scalar(0,0,255),1);
    cv::rectangle(display, cv::Point(10, 30), cv::Point(550, 60), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(display, "Camera Trajectory (RED SQUARE)", cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    char text[100];
    sprintf(text, "x = %02fm y = %02fm z = %02fm", x, y, z);
    cv::putText(display, text, cv::Point(10, 50), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);


    //now for gt odom
    x = -gt_odom.pose.pose.position.x;
    y = gt_odom.pose.pose.position.y;
    z = gt_odom.pose.pose.position.z;
    //here we update the odom repub to the display mat
    //we use 10 for scale    
    x_display = (x * scale) + x_offset;
    y_display = (y * scale) + y_offset;
    //add odom to cv mat
    cv::rectangle(display, cv::Point(y_display, x_display), cv::Point(y_display+10, x_display+10), cv::Scalar(0,255,0),1);
    cv::rectangle(display, cv::Point(10, 100), cv::Point(550, 130), CV_RGB(0,0,0), CV_FILLED);
    cv::putText(display, "Camera GT Trajectory (GREEN SQUARE)", cv::Point(10, 100), cv::FONT_HERSHEY_COMPLEX, 0.6, CV_RGB(255, 255, 255), 1);
    char text1[100];
    sprintf(text1, "x = %.2f y = %.2f z = %.2f", x, y, z);
    cv::putText(display, text1, cv::Point(10, 120), cv::FONT_HERSHEY_COMPLEX, 0.6, cv::Scalar::all(255), 1);

    display_mutex.unlock();


    //draw each object
    std::vector<SceneObject> scene_objects = scene->get_scene_objects();

    for(SceneObject& scene_object : scene_objects) {
        int x = scene_object.pose.x;
        int y = scene_object.pose.y;

        std::string frame = scene->get_frame_id();
        std::string child_frame = scene->get_child_frame_id();

        cv::Point3d p(x, y, 0);
        cv::Point3d transformed_point;
        ros::Time t = scene->get_ros_time();

        VDO_SLAM::utils::apply_transform(child_frame,frame, p, &transformed_point,t);


        int x_display =  static_cast<int>(transformed_point.x*scale) + x_offset;
        int y_display =  static_cast<int>(transformed_point.y*scale) + y_offset;


        // TODO: use colour conversion function
        switch (scene_object.tracking_id) {

            case 1:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // orange
                break;
            case 2:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,125,125), 5); // green
                break;
            case 3:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0, 255, 255), 5); // yellow
                break;
            case 4:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(0,0,255), 5); // pink
                break;
            case 5:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,0), 5); // cyan (yellow green 47,255,173)
                break;
            case 6:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(128, 0, 128), 5); // purple
                break;
            case 7:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(255,255,255), 5);  // white
                break;
            case 8:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(196,228,255), 5); // bisque
                break;
            case 9:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(180, 105, 255), 5);  // blue
                break;
            case 10:
                cv::circle(display, cv::Point(x_display, y_display), 2, CV_RGB(42,42,165), 5);  // brown
                break;
            default:
                ROS_WARN_STREAM("No case for this segmentaion index yet");
                break;
        }

    }


}


