#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <tf2_msgs/TFMessage.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <functional>


#include <string>
#include <vector>
#include <algorithm>
#include <map>

typedef const sensor_msgs::ImageConstPtr& ImageConst;

class VdoBagPlayback {

    public:
        VdoBagPlayback(std::string& _out_file):
            out_file(_out_file) {
                bag.open(out_file, rosbag::bagmode::Write);
                ROS_INFO_STREAM("Making output bag file: " << out_file);

            };

        ~VdoBagPlayback() {
            close();
        }

        void close() {
            bag.close();
            ROS_INFO_STREAM("Closing bag file");
        }

        void odom_callback(const nav_msgs::OdometryConstPtr& msg) {
            bag.write("/odom", msg->header.stamp, *msg);
        }
        
        void tf_callback(const tf2_msgs::TFMessageConstPtr& msg) {
            bag.write("/tf", msg->transforms[0].header.stamp, *msg);
        }

        void tf_static_callback(const tf2_msgs::TFMessageConstPtr& msg) {
            bag.write("/tf_static", msg->transforms[0].header.stamp, *msg);
        }

        void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {
            bag.write("/camera/camera_info",msg->header.stamp, *msg);
        }

        void image_synch_callback(ImageConst raw_image, ImageConst mask, ImageConst flow, ImageConst depth) {
            ROS_INFO_STREAM("In image sunch callback");
            ros::Time t = raw_image->header.stamp;
            bag.write("/camera/rgb/image_raw",t, *raw_image);
            bag.write("/camera/mask/image_raw",t, *mask);
            bag.write("/camera/flow/image_raw",t, *flow);
            bag.write("/camera/depth/image_raw",t, *depth);
        }

    private:

        std::string out_file;
        rosbag::Bag bag;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vdo_bag_generation");
    ros::NodeHandle n;

    std::string out_file_name, video_stream_namespace;
    n.param<std::string>("/out_file", out_file_name, "out");
    n.param<std::string>("/video_stream_namespace", video_stream_namespace, "/gmsl/A2/");

    std::string input_video_topic = "/vdoslam/input/camera/rgb/image_raw";
    std::string camera_info_topic = video_stream_namespace + "camera_info";
    std::string mask_rcnn_topic = "/vdoslam/input/camera/mask/image_raw";
    std::string flow_net_topic = "/vdoslam/input/camera/flow/image_raw";
    std::string monodepth_topic = "/vdoslam/input/camera/depth/image_raw";

    std::string odom_topic = "/odometry";
    std::string map_topic = "/map";
    std::string tf_static_topic = "/tf_static";
    std::string tf_topic = "/tf";

    VdoBagPlayback playback(out_file_name);
    //these messages can can come in at anytime
    ros::Subscriber sub_odom = n.subscribe(odom_topic, 1000, &VdoBagPlayback::odom_callback, &playback);
    ros::Subscriber sub_tf_static = n.subscribe(tf_static_topic, 1000, &VdoBagPlayback::tf_static_callback, &playback);
    ros::Subscriber sub_tf = n.subscribe(tf_static_topic, 1000, &VdoBagPlayback::tf_static_callback, &playback);
    ros::Subscriber sub_camera_info = n.subscribe(camera_info_topic, 1000, &VdoBagPlayback::camera_info_callback, &playback);

    //these ones we must synchronize to a single time
    //TODO: add subscribers to visualser topics for flow and maskrcnn
    message_filters::Subscriber<sensor_msgs::Image> image_raw_sub(n, input_video_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> mask_rcnn_sub(n, mask_rcnn_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> flow_sub(n, flow_net_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, monodepth_topic, 1);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(image_raw_sub,
            mask_rcnn_sub,flow_sub, depth_sub,  10);

    sync.registerCallback(boost::bind(&VdoBagPlayback::image_synch_callback, &playback, _1, _2, _3, _4));

    ros::spin();
    // playback.close();

}