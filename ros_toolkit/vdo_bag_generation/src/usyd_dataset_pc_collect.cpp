#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <realtime_vdo_slam/VdoInput.h>
#include <lidar_camera_projection/ImagePixelDepth.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2_msgs/TFMessage.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <midas_ros/MidasDepth.h>
#include <midas_ros/MidasDepthInterface.hpp>

#include <minisam/core/LossFunction.h>
#include <minisam/core/Eigen.h>
#include <minisam/core/Factor.h>
#include <minisam/core/FactorGraph.h>
#include <minisam/core/Variables.h>
#include <minisam/nonlinear/LevenbergMarquardtOptimizer.h>


#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <functional>
#include <opencv2/core.hpp>

#include "colour_map.h"

#include <string>
#include <vector>
#include <algorithm>
#include <memory>
#include <map>

typedef const sensor_msgs::ImageConstPtr& ImageConst;

//wrapper for camera information
struct CameraInformation {
    typedef std::unique_ptr<CameraInformation> CameraInformationPtr;

    std::string topic;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat map1, map2, modified_camera_matrix;
    sensor_msgs::CameraInfo camera_info_msg;

};

class ExpCurveFittingFactor : public minisam::Factor {
    private:
        Eigen::Vector2d p_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ExpCurveFittingFactor(minisam::Key key, const Eigen::Vector2d& point,
                            const std::shared_ptr<minisam::LossFunction>& lossfunc);
                            
        virtual ~ExpCurveFittingFactor() = default;

        // deep copy function
        std::shared_ptr<Factor> copy() const;

        // error function
        // clang-format off
        Eigen::VectorXd error(const minisam::Variables& values) const override;

        // jacobians function
        std::vector<Eigen::MatrixXd> jacobians(const minisam::Variables& values) const;
        // clang-format on
};

ExpCurveFittingFactor::ExpCurveFittingFactor(minisam::Key key, const Eigen::Vector2d& point,
                            const std::shared_ptr<minisam::LossFunction>& lossfunc)
    :   minisam::Factor(1, std::vector<minisam::Key>{key}, lossfunc), p_(point) {}

std::shared_ptr<minisam::Factor> ExpCurveFittingFactor::copy() const {
    return std::shared_ptr<Factor>(new ExpCurveFittingFactor(*this));
}

Eigen::VectorXd ExpCurveFittingFactor::error(const minisam::Variables& values) const {
    const Eigen::Vector2d& params = values.at<Eigen::Vector2d>(keys()[0]);
    return (Eigen::VectorXd(1) << params(0) * p_(0) + params(1) - p_(1))
        .finished();
}

std::vector<Eigen::MatrixXd> ExpCurveFittingFactor::jacobians(const minisam::Variables& values) const {
    const Eigen::Vector2d& params = values.at<Eigen::Vector2d>(keys()[0]);
    return std::vector<Eigen::MatrixXd>{
        (Eigen::MatrixXd(1, 2) <<
            1,
            1)
        .finished()};
}


class UsydDataPCCollectPlayBack {

    public:
        UsydDataPCCollectPlayBack(std::string& _out_file):
            out_file(_out_file)
        {
            // bag.open(out_file, rosbag::bagmode::Write);
            ROS_INFO_STREAM("Making output bag file: " << out_file);

            std::vector<std::string> topics{"/map", "/odom", "/tf", "/ublox_gps/fix", 
                "/tf_static", "/camera/camera_info", "/camera/sync", "velodyne/points"};

            ros::Time time = ros::Time::now();

            for (int i = 0; i < topics.size(); i++) {
                std::string topic = topics[i];
                TimingInfoPtr timing_info = std::make_shared<TimingInfo>();

                topic_timing_map.emplace(topic, timing_info);
            }

            midas_depth  = std::make_shared<midas_ros::MidasDepthInterface>(nh);
            midas_depth->start_service();
            init_distortion_params();
            loss = minisam::CauchyLoss::Cauchy(1.0);


        }

        ~UsydDataPCCollectPlayBack() {
            close();
        }

        

        void close() {
            // bag.close();
            ROS_INFO_STREAM("Closing bag file");
        }

        ros::Time get_timing(const std::string& topic, const ros::Time& msg_time) {
            TimingInfoPtr timing_info = topic_timing_map[topic];
            ros::Duration dt;

            //the frist time is actually when we start so dt should be 0 and video timing becomes now
            // everything else is relative to this
            if (!timing_info->data_recieved) {
                dt = ros::Duration(0.01);
                timing_info->video_time = ros::Time::now();

                if (timing_info->video_time <= ros::TIME_MIN) {
                    timing_info->video_time = ros::TIME_MIN;
                }

                timing_info->data_recieved = true;
            }
            else {
                dt = (msg_time - timing_info->prev_msg_time);
            }
    

            ros::Time save_time = timing_info->video_time + dt;

            if (save_time <= ros::TIME_MIN) {
                save_time = ros::TIME_MIN;
            } 
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

        void gps_info_callback(const sensor_msgs::NavSatFixConstPtr& msg) {
            ros::Time save_time = get_timing("/camera/camera_info", msg->header.stamp);
            bag.write("/gps",save_time, *msg);
        }

        void pc_projected_image_callback(ImageConst& image_projected_msg) {

        }

        void image_projected_points_callback(ImageConst& rgb_msg, const lidar_camera_projection::ImagePixelDepthConstPtr& projected_pts) {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb_image = cv_ptr->image;
            std::vector<cv::Point2f> distorted_lidar_points;

            undistort_depth_image(rgb_image, rgb_image);


            for(const lidar_camera_projection::PixelDepth& pixel_depth : projected_pts->data) {
                double dis = pow(pixel_depth.x * pixel_depth.x + pixel_depth.y * pixel_depth.y + pixel_depth.z * pixel_depth.z, 0.5);
                int range = std::min(float(round((dis / 50) * 149)), (float) 149.0);

                cv::Point p;
                p.x = static_cast<float>(pixel_depth.pixel_x);
                p.y = static_cast<float>(pixel_depth.pixel_y);

                distorted_lidar_points.push_back(p);


            }
            std::vector<cv::Point2f> correct_lidar_points;
            undistort_points(distorted_lidar_points,correct_lidar_points);

            std::vector<lidar_camera_projection::PixelDepth> corrected_projections;

            cv::Mat disp;
            bool mono_depth_success = midas_depth->analyse(rgb_image, disp);

            // disp.convertTo(disp, CV_16SC1);
            ROS_INFO_STREAM("No points " << correct_lidar_points.size());
            minisam::FactorGraph factor_graph;
            for(int i = 0; i < correct_lidar_points.size(); i++) {
                if (i%2 == 0) {
                    continue;
                }
                cv::Point p = correct_lidar_points[i];
                lidar_camera_projection::PixelDepth pixel_depth = projected_pts->data[i];

                double dis = pow(pixel_depth.x * pixel_depth.x + pixel_depth.y * pixel_depth.y + pixel_depth.z * pixel_depth.z, 0.5);
                int range= std::min(float(round((dis / 50) * 149)), (float) 149.0);
                double range_d = static_cast<double>(range);

                lidar_camera_projection::PixelDepth correct_pixel;
                correct_pixel.pixel_x = p.x;
                correct_pixel.pixel_y = p.y;

                correct_pixel.x = pixel_depth.x;
                correct_pixel.y = pixel_depth.y;
                correct_pixel.z = pixel_depth.z;

                corrected_projections.push_back(correct_pixel);

                cv::circle(rgb_image,
                       cv::Point(correct_pixel.pixel_x, correct_pixel.pixel_y), 3,
                       CV_RGB(255 * colmap[range][0], 255 * colmap[range][1], 255 * colmap[range][2]), -1);

                double disp_map_range = (double)disp.at<uint16_t>(correct_pixel.pixel_y, correct_pixel.pixel_x)/1000.0;
                ROS_INFO_STREAM(range << " " << disp_map_range);
                factor_graph.add(ExpCurveFittingFactor(minisam::key('p', 0), Eigen::Vector2d(disp_map_range,
                                                range_d), loss));
            }


            minisam::Variables init_values;

            init_values.add(minisam::key('p', 0), Eigen::Vector2d(0, 0));

            // optimize!
            minisam::LevenbergMarquardtOptimizerParams opt_param;
            opt_param.verbosity_level = minisam::NonlinearOptimizerVerbosityLevel::ITERATION;
            opt_param.lambda_max = 1e10; //idk what this should be
            minisam::LevenbergMarquardtOptimizer opt(opt_param);

            minisam::Variables values;
            auto status = opt.optimize(factor_graph, init_values, values);
            if(status == minisam::NonlinearOptimizationStatus::SUCCESS) {
                Eigen::Vector2d result = values.at<Eigen::Vector2d>(minisam::key('p', 0));
                double s = result[0];
                double t = result[1];
                ROS_INFO_STREAM("s: " << s << " t: " << t);

            }

            cv::imshow("Projection", rgb_image);
            cv::waitKey(1);

            cv::imshow("Disp", disp);
            cv::waitKey(1);
        }

        // void vdo_input_callback(const realtime_vdo_slam::VdoInputConstPtr& input) {
        //     ros::Time save_time = get_timing("/camera/sync", input->header.stamp);
        //     ROS_INFO_STREAM("Synch callback time now " << save_time);


        //     bag.write("/camera/rgb/image_raw",save_time, input->rgb);

        //     bag.write("/camera/mask/image_raw",save_time, input->mask);
        //     // bag.write("/camera/mask/colour_mask",save_time, *mask_viz);

        //     bag.write("/camera/flow/image_raw",save_time, input->flow);
        //     // bag.write("/camera/flow/colour_map",save_time, *flow_viz);

        //     bag.write("/camera/depth/image_raw",save_time, input->depth);

        //     bag.write("/camera/all", save_time, *input);


        // }

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

        typedef std::shared_ptr<UsydDataPCCollectPlayBack::TimingInfo> TimingInfoPtr;

        midas_ros::MidasDepthInterfacePtr midas_depth;
        std::string out_file;
        rosbag::Bag bag;
        ros::NodeHandle nh;
        CameraInformation camera_info;

        std::shared_ptr<minisam::LossFunction> loss;
        
        void init_distortion_params();
        void undistort_depth_image(const cv::Mat& depth, cv::Mat& depth_undistorted);
        // void undistort_lidar_points(const cv::Mat& points, cv::Mat& undistorted_points);
        void undistort_points(const std::vector<cv::Point2f>& distorted_points, std::vector<cv::Point2f>& undistorted_points);
    

        std::map<std::string, UsydDataPCCollectPlayBack::TimingInfoPtr> topic_timing_map;

        ros::Time update_timing(const std::string& topic, const ros::Time& msg_time);

};

void UsydDataPCCollectPlayBack::init_distortion_params() {
    std::string input_camera_info_topic = "gmsl/A0/camera_info";
    ROS_INFO_STREAM("Waiting for camera info topic: " << input_camera_info_topic);
    sensor_msgs::CameraInfoConstPtr info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(input_camera_info_topic);

    //for undistortion
    camera_info.camera_info_msg = *info;

    uint32_t image_width = camera_info.camera_info_msg.width;
    uint32_t image_height = camera_info.camera_info_msg.height;

    cv::Size image_size = cv::Size(image_width, image_height);

    if (camera_info.camera_info_msg.distortion_model == "rational_polynomial") {
        camera_info.camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info.camera_info_msg.K[0]);
        camera_info.dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info.camera_info_msg.D[0]);
    }
    else if (camera_info.camera_info_msg.distortion_model == "equidistant") {
        camera_info.camera_matrix = cv::Mat(3, 3, CV_64F, &camera_info.camera_info_msg.K[0]);
        camera_info.dist_coeffs = cv::Mat(4, 1, CV_64F, &camera_info.camera_info_msg.D[0]);

        //cv::Mat scaled_camera_matrix = camera_matrix *
        // camera_info.camera_matrix.at<double>(2, 2) = 1.;

        //cv::Mat output_image;
        cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64F);

        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_info.camera_matrix, camera_info.dist_coeffs, image_size,
                                                                identity_mat, camera_info.modified_camera_matrix);

        cv::fisheye::initUndistortRectifyMap(camera_info.camera_matrix,
                                            camera_info.dist_coeffs,
                                            identity_mat,
                                            camera_info.modified_camera_matrix,
                                            image_size,
                                            CV_16SC2,
                                            camera_info.map1, camera_info.map2);
        cv::convertMaps(camera_info.map1, camera_info.map2, camera_info.map1, camera_info.map2, CV_32FC1);
    }
}

void UsydDataPCCollectPlayBack::undistort_depth_image(const cv::Mat& depth, cv::Mat& depth_undistorted) {
     if (camera_info.camera_info_msg.distortion_model == "rational_polynomial") {
        cv::undistort(depth, depth_undistorted, camera_info.camera_matrix, camera_info.dist_coeffs);
    }
    else if (camera_info.camera_info_msg.distortion_model == "equidistant") {
        // cv::remap(input, undistorted, camera_info.map1, camera_info.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
        cv::fisheye::undistortImage(depth, depth_undistorted, camera_info.camera_matrix, camera_info.dist_coeffs, camera_info.modified_camera_matrix);
    }

}

/**
 * @brief Undistort as set of points so we can map the undisroted depth map to the points from the lidar
 * which are distorted. Dst is the undistorted image and src is the original set of points from the lidar (distorted).
 * 
 * dst(x,y) = src(map1(x,y), map2(x,y)) so out undisroted points will be (map1(x,y), map2(x,y))
 * 
 * @param points 
 * @param undistorted_points 
 */
// void UsydDataPCCollectPlayBack::undistort_lidar_points(const cv::Mat& points, cv::Mat& undistorted_points) {
//     undistorted_points.at<int>(0,0) = static_cast<int>(camera_info.map1.at<float>(points.at<int>(0,0), points.at<int>(0,1)));
//     undistorted_points.at<int>(0,1) = static_cast<int>(camera_info.map2.at<float>(points.at<int>(0,0), points.at<int>(0,1)));
// }

void UsydDataPCCollectPlayBack::undistort_points(const std::vector<cv::Point2f>& distorted_points, std::vector<cv::Point2f>& undistorted_points) {
    cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64F);
    cv::fisheye::undistortPoints(distorted_points, undistorted_points, camera_info.camera_matrix, camera_info.dist_coeffs, identity_mat, camera_info.modified_camera_matrix);	
 }
    


int main(int argc, char **argv)
{
    ros::init(argc, argv, "usyd_dataset_pc_collect");
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
    std::string gps_topic = "/ublox_gps/fix";
    std::string point_cloud_topic = "/velodyne/front/corrected";

    //TODO: add gps - not needed for paper (we just need odom)

    UsydDataPCCollectPlayBack playback(out_file_name);
    //these messages can can come in at anytime
    // ros::Subscriber sub_odom = n.subscribe(odom_topic, 100, &UsydDataPCCollectPlayBack::odom_callback, &playback);
    // ros::Subscriber sub_map = n.subscribe(map_topic, 100, &UsydDataPCCollectPlayBack::map_callback, &playback);
    // ros::Subscriber sub_tf_static = n.subscribe(tf_static_topic, 100, &UsydDataPCCollectPlayBack::tf_static_callback, &playback);
    // ros::Subscriber sub_tf = n.subscribe(tf_topic, 100, &UsydDataPCCollectPlayBack::tf_callback, &playback);
    // ros::Subscriber sub_camera_info = n.subscribe(camera_info_topic, 100, &UsydDataPCCollectPlayBack::camera_info_callback, &playback);
    // ros::Subscriber gps_sub_info = n.subscribe(gps_topic, 100, &UsydDataPCCollectPlayBack::gps_info_callback, &playback);

    ros::Subscriber sub_lidar_camera_projection = n.subscribe("/gmsl/A0/image_color/lidar_projected", 100, &UsydDataPCCollectPlayBack::pc_projected_image_callback, &playback);

    // //these ones we must synchronize to a single time
    message_filters::Subscriber<sensor_msgs::Image> image_raw_sub(n, "/gmsl/A0/image_color", 10);
    message_filters::Subscriber<lidar_camera_projection::ImagePixelDepth> pixel_depth(n, "/gmsl/A0/image_color/image_pixel_depth", 10);
    // message_filters::Subscriber<sensor_msgs::Image> mask_rcnn_viz_sub(n, mask_rcnn_viz_topic, 10);

    // message_filters::Subscriber<sensor_msgs::Image> flow_sub(n, flow_net_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> flow_viz_sub(n, flow_net_viz_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, monodepth_topic, 10);

    message_filters::TimeSynchronizer<sensor_msgs::Image, lidar_camera_projection::ImagePixelDepth> sync(image_raw_sub,
                                                                                                       pixel_depth,  20);


    ///velodyne/front/corrected

    sync.registerCallback(boost::bind(&UsydDataPCCollectPlayBack::image_projected_points_callback, &playback, _1, _2));
    // ros::Subscriber vdo_input_sub = n.subscribe("/vdoslam/input/all",100, &UsydDataPCCollectPlayBack::vdo_input_callback, &playback );

    ros::spin();
    // playback.close();

}