#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <lidar_camera_projection/ImagePixelDepth.h>
// #include <MonoDepth.hpp>
#include <MonoDepth.hpp>
#include <tf2_msgs/TFMessage.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <functional>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>


#include <ros/package.h>


#include <string>
#include <vector>
#include <algorithm>
#include <map>
#include <yaml-cpp/yaml.h>
#include <memory>

typedef const sensor_msgs::ImageConstPtr ImageConstPtr;
typedef const lidar_camera_projection::ImagePixelDepthConstPtr ImagePixelDepthConstPtr;

class DepthAnalyser {

    typedef message_filters::Subscriber<sensor_msgs::Image> RawImageMF;
    typedef message_filters::Subscriber<lidar_camera_projection::ImagePixelDepth> ImagePixelMF;
    typedef message_filters::TimeSynchronizer<sensor_msgs::Image, lidar_camera_projection::ImagePixelDepth> TimeSynch;


    public:
        DepthAnalyser(std::string& _video_namespace, ros::NodeHandle& _nh);
        bool is_complete();
        void save_data();

        void depth_pixel_callback(ImageConstPtr& image_msg, ImagePixelDepthConstPtr& pixel_depth_msg);

        struct PixelDepthStruct {
            int pixel_x, pixel_y;

            float lidar_x, lidar_y, lidar_z; //3D coordinates from the lidar
            u_int16_t mono_depth; //Depth from the monodepth algorithm
            float lidar_depth; //Total depth from the lidar will be sqrt(lidar_x^2 + lidar_y^2 + lidar_z^2)
        };

        struct ImagePixelDepthStruct {
            int frame_id; //sequence number < no_images
            int time; //in seconds

            std::vector<PixelDepthStruct> pixel_depths;
        };




    private:
        ros::NodeHandle nh;
        //these ones we must synchronize to a single time
        RawImageMF raw_image_sub;
        ImagePixelMF pixel_depth_sub;
        TimeSynch sync;


        // image_transport::Subscriber coloured_pc_sub;

        // ros::Subscriber pixel_depth_sub;

        std::string video_namespace;
        MonoDepth mono_depth;

        int no_images, time_delay;

        ros::Time last_image_captrure;
        int count;

        std::string yaml_file;
        YAML::Node node;
    

        
};


DepthAnalyser::DepthAnalyser(std::string& _video_namespace, ros::NodeHandle& _nh) :
    nh(_nh),
    video_namespace(_video_namespace),
    mono_depth(nh),
    sync(raw_image_sub, pixel_depth_sub, 20) {

        nh.param<int>("/no_images", no_images, 10); //how many images to take before stopping
        nh.param<int>("/time_delay", time_delay, 3); //how many seconds to delay before capturing between images

        // std::string pixel_depth_topic = video_namespace + std::string("/image_color/image_pixel_depth");
        // std::string raw_image_topic = video_namespace + std::string("/image_color");
        // // pixel_depth_sub = nh.subscribe(pixel_depth_topic, 20, &DepthAnalyser::depth_pixel_callback, this);
        // raw_image_sub = std::make_unique<RawImageMF>(nh, raw_image_topic, 10);
        // pixel_depth_sub = std::make_unique<ImagePixelMF>(nh, pixel_depth_topic, 10);
        pixel_depth_sub.subscribe(nh, video_namespace + std::string("/image_color/image_pixel_depth"), 10);
        raw_image_sub.subscribe(nh, video_namespace + std::string("/image_color"), 10);

        // sync = std::make_unique<TimeSynch>(*raw_image_sub, *pixel_depth_sub, 20);
        sync.registerCallback(boost::bind(&DepthAnalyser::depth_pixel_callback, this, _1, _2));
                                                                                                       

        std::string path = ros::package::getPath("lidar_monodepth2");
        yaml_file = path + "/results/depth_mod.yaml";

        ROS_INFO_STREAM("Saving results to " << yaml_file);


        mono_depth.start_service();
        MonoDepth::wait_for_mono_services();
        last_image_captrure = ros::TIME_MIN;
        count = 0;
    }


void DepthAnalyser::depth_pixel_callback(ImageConstPtr& image_msg, ImagePixelDepthConstPtr& pixel_depth_msg) {
    ros::Time callback_time = image_msg->header.stamp;
    bool to_capture = false;

    if (last_image_captrure == ros::TIME_MIN) {
        to_capture = true;
        last_image_captrure = callback_time;
    }
    else {
        int dt = static_cast<int>((callback_time - last_image_captrure).toSec());
        if (dt > time_delay) {
            to_capture = true;
            last_image_captrure = callback_time;
        }
    }

    if (to_capture) {
        ROS_INFO_STREAM("Capturing image [" << count << "] at time " << callback_time);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*image_msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat src, dst;

        src = cv_ptr->image;


        ImagePixelDepthStruct image_depth_info;
        image_depth_info.frame_id = count;
        image_depth_info.time = static_cast<int>(callback_time.toSec());

        mono_depth.analyse_image(src, dst);
        //this is the same preprocessing we do in the VdoSlamInput struct (RealTimeVdoSlam.hpp)
        // dst.convertTo(dst, CV_32F);
        ROS_INFO_STREAM(dst.size());

        for (const lidar_camera_projection::PixelDepth& info : pixel_depth_msg->data) {
            PixelDepthStruct depth_struct;
            depth_struct.lidar_x = info.x;
            depth_struct.lidar_y = info.y;
            depth_struct.lidar_z = info.z;
            depth_struct.lidar_depth = std::sqrt(depth_struct.lidar_x *  depth_struct.lidar_x + 
                                                depth_struct.lidar_y *  depth_struct.lidar_y + 
                                                depth_struct.lidar_z *  depth_struct.lidar_z);

            depth_struct.pixel_x = info.pixel_x;
            depth_struct.pixel_y = info.pixel_y;

            u_int16_t depth = dst.at<u_int16_t>(info.pixel_x, info.pixel_y);
            depth_struct.mono_depth = depth;

            image_depth_info.pixel_depths.push_back(depth_struct);
        }

        node[std::to_string(count)] = image_depth_info;
        count++;

    }
    
}

bool DepthAnalyser::is_complete() {
    return count >= no_images;
}

void DepthAnalyser::save_data() {
    std::ofstream fout(yaml_file);
    fout << node;
    ROS_INFO_STREAM("saved data");
}

namespace YAML {
    template<>
    struct convert<DepthAnalyser::ImagePixelDepthStruct> {
        static Node encode(const DepthAnalyser::ImagePixelDepthStruct& rhs) {
            Node node;

            node["frame_id"]  = rhs.frame_id;
            node["time"]  = rhs.time;

            for (const DepthAnalyser::PixelDepthStruct& depth : rhs.pixel_depths) {
                Node pixel_depth_node;
                pixel_depth_node["lidar_x"] = depth.lidar_x;
                pixel_depth_node["lidar_y"] = depth.lidar_y;
                pixel_depth_node["lidar_z"] = depth.lidar_z;

                pixel_depth_node["pixel_x"] = depth.pixel_x;
                pixel_depth_node["pixel_y"] = depth.pixel_y;

                pixel_depth_node["lidar_depth"] = depth.lidar_depth;

                pixel_depth_node["mono_depth"] = depth.mono_depth;
                node["data"].push_back(pixel_depth_node);
            }
            return node;
        }

        static bool decode(const Node& node, DepthAnalyser::ImagePixelDepthStruct& rhs) {
            // if(!node.IsSequence() || node.size() != 3) {
            // return false;
            // }

            for(const Node& pixel_depth_node : node["data"]) {
                DepthAnalyser::PixelDepthStruct depth;
                depth.lidar_x = pixel_depth_node["lidar_x"].as<float>();
                depth.lidar_y = pixel_depth_node["lidar_y"].as<float>();
                depth.lidar_z = pixel_depth_node["lidar_z"].as<float>();

                depth.pixel_x = pixel_depth_node["pixel_x"].as<int>();
                depth.pixel_y = pixel_depth_node["pixel_y"].as<int>();

                depth.lidar_depth = pixel_depth_node["lidar_depth"].as<float>();

                depth.mono_depth = pixel_depth_node["mono_depth"].as<u_int16_t>();

                rhs.pixel_depths.push_back(depth);
            }

            rhs.frame_id = node["frame_id"].as<int>();
            rhs.time = node["time"].as<int>();
            return true;
        }
    };
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_monodepth2_node");
    ros::NodeHandle n;

    std::string video_stream_namespace;
    n.param<std::string>("/video_stream_namespace", video_stream_namespace, "/gmsl/A1/");
    //subscribe to video playback from multi_camera_lidar node (ACFR-ITS code)
    //subscribe to colored_pointcloud as well for images
    //process with monodepth2
    //save both images in results folder for checking

    //for all Pixel depths in ImagePixelDepth
        //get associated predicted depth
        //save as yaml file
    DepthAnalyser analyser(video_stream_namespace, n);

    while (ros::ok() && !analyser.is_complete()) {
        ros::spinOnce();
    }

    analyser.save_data();
    
    // playback.close();

}