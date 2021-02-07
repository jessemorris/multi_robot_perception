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

                std::vector<std::string> topics{"/map", "/odom", "/tf", "/tf_static", "/camera/camera_info", "/camera/sync"};

                ros::Time time = ros::Time::now();

                for (int i = 0; i < topics.size(); i++) {
                    std::string topic = topics[i];
                    TimingInfoPtr timing_info = std::make_shared<TimingInfo>();

                    topic_timing_map.emplace(topic, timing_info);
                }


            };

        ~VdoBagPlayback() {
            close();
        }

        void close() {
            bag.close();
            ROS_INFO_STREAM("Closing bag file");
        }

        ros::Time get_timing(const std::string& topic, const ros::Time& msg_time) {
            TimingInfoPtr timing_info = topic_timing_map[topic];
            ros::Duration dt;

            //the frist time is actually when we start so dt should be 0 and video timing becomes now
            // everything else is relative to this
            if (!timing_info->data_recieved) {
                dt = ros::Duration(0);
                timing_info->video_time = ros::Time::now();
                timing_info->data_recieved = true;
            }
            else {
                dt = (msg_time - timing_info->prev_msg_time);
            }
    

            ros::Time save_time = timing_info->video_time + dt;

            timing_info->video_time = save_time;
            timing_info->prev_msg_time = msg_time;

            return save_time;
        }

        void map_callback(const nav_msgs::OdometryConstPtr& msg) {
            ros::Time save_time = get_timing("/map", msg->header.stamp);
            bag.write("/map", save_time, *msg);
        }

        void odom_callback(const nav_msgs::OdometryConstPtr& msg) {
            ros::Time save_time = get_timing("/odom", msg->header.stamp);
            bag.write("/odom", save_time, *msg);
        }
        
        void tf_callback(const tf2_msgs::TFMessageConstPtr& msg) {
            ros::Time save_time = get_timing("/tf", msg->transforms[0].header.stamp);
            bag.write("/tf", save_time, *msg);
        }

        void tf_static_callback(const tf2_msgs::TFMessageConstPtr& msg) {
            ros::Time save_time = get_timing("/tf_static", msg->transforms[0].header.stamp);
            bag.write("/tf_static", save_time, *msg);
        }

        void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& msg) {
            ros::Time save_time = get_timing("/camera/camera_info", msg->header.stamp);
            bag.write("/camera/camera_info",save_time, *msg);
        }

        void image_synch_callback(ImageConst raw_image, ImageConst mask, ImageConst mask_viz, ImageConst flow, ImageConst flow_viz, ImageConst depth) {
            ROS_INFO_STREAM("Synch callback time " << raw_image->header.stamp);
            // ros::Duration dt = (raw_image->header.stamp - previous_msg_time);
            // ros::Time save_time = video_time + dt;  
            ros::Time save_time = get_timing("/camera/sync", raw_image->header.stamp);
            ROS_INFO_STREAM("Synch callback time now " << save_time);   
            bag.write("/camera/rgb/image_raw",save_time, *raw_image);

            bag.write("/camera/mask/image_raw",save_time, *mask);
            bag.write("/camera/mask/colour_mask",save_time, *mask_viz);

            bag.write("/camera/flow/image_raw",save_time, *flow);
            bag.write("/camera/flow/colour_map",save_time, *flow_viz);

            bag.write("/camera/depth/image_raw",save_time, *depth);


        }

    private:

        //the current rostime for this recording. the result will be ros::Time::now() of previous image_synch_callback (ie.
        // the last time a image msg was saved) + dt (the walltime between actual image msgs)
        typedef ros::Time VideoTime;
         //time rostime associated with the header of the previous incoming image
        //need this to determine the dt between the new msg and the previous now
        typedef ros::Time PrevMsgTime;

        struct TimingInfo {

            VideoTime video_time;
            PrevMsgTime prev_msg_time;

            //set to false, we only want to count timing from the first msg, otherwise make dt 0;
            bool data_recieved;

            TimingInfo() :
                data_recieved(false) {}


        };

        typedef std::shared_ptr<VdoBagPlayback::TimingInfo> TimingInfoPtr;


        std::string out_file;
        rosbag::Bag bag;

        std::map<std::string, VdoBagPlayback::TimingInfoPtr> topic_timing_map;

        ros::Time update_timing(const std::string& topic, const ros::Time& msg_time);


};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vdo_bag_generation");
    ros::NodeHandle n;

    std::string out_file_name, video_stream_namespace;
    n.param<std::string>("/out_file", out_file_name, "vdo_out.bag");
    n.param<std::string>("/video_stream_namespace", video_stream_namespace, "/gmsl/A1/");

    std::string input_video_topic = "/vdoslam/input/camera/rgb/image_raw";
    std::string camera_info_topic = video_stream_namespace + "camera_info";
    std::string mask_rcnn_topic = "/vdoslam/input/camera/mask/image_raw";
    std::string flow_net_topic = "/vdoslam/input/camera/flow/image_raw";
    std::string monodepth_topic = "/vdoslam/input/camera/depth/image_raw";


    //topics for viz
    std::string mask_rcnn_viz_topic = "/vdoslam/input/camera/mask/colour_mask";
    std::string flow_net_viz_topic = "/vdoslam/input/camera/flow/colour_map";

    //we get these from the localiser node not the original /odom in the bag file. 
    std::string odom_topic = "/localiser/odometry";
    std::string map_topic = "/localiser/map";
    std::string tf_static_topic = "/tf_static";
    std::string tf_topic = "/tf";

    VdoBagPlayback playback(out_file_name);
    //these messages can can come in at anytime
    ros::Subscriber sub_odom = n.subscribe(odom_topic, 100, &VdoBagPlayback::odom_callback, &playback);
    ros::Subscriber sub_map = n.subscribe(map_topic, 100, &VdoBagPlayback::map_callback, &playback);
    ros::Subscriber sub_tf_static = n.subscribe(tf_static_topic, 100, &VdoBagPlayback::tf_static_callback, &playback);
    ros::Subscriber sub_tf = n.subscribe(tf_topic, 100, &VdoBagPlayback::tf_callback, &playback);
    ros::Subscriber sub_camera_info = n.subscribe(camera_info_topic, 100, &VdoBagPlayback::camera_info_callback, &playback);

    //these ones we must synchronize to a single time
    message_filters::Subscriber<sensor_msgs::Image> image_raw_sub(n, input_video_topic, 10);

    message_filters::Subscriber<sensor_msgs::Image> mask_rcnn_sub(n, mask_rcnn_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> mask_rcnn_viz_sub(n, mask_rcnn_viz_topic, 10);

    message_filters::Subscriber<sensor_msgs::Image> flow_sub(n, flow_net_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> flow_viz_sub(n, flow_net_viz_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, monodepth_topic, 10);

    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, 
                                      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync(image_raw_sub,
                                                                                                       mask_rcnn_sub, mask_rcnn_viz_sub, 
                                                                                                       flow_sub, flow_viz_sub, 
                                                                                                       depth_sub,  20);

    sync.registerCallback(boost::bind(&VdoBagPlayback::image_synch_callback, &playback, _1, _2, _3, _4, _5, _6));

    ros::spin();
    // playback.close();

}