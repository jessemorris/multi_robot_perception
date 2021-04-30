#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
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
#include <minisam/linear/SparseCholesky.h>
#include <minisam/linear/DenseCholesky.h>
#include <minisam/linear/ConjugateGradient.h>
// #include <Eigen/Core>


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
    return (Eigen::VectorXd(1) <<  p_(1) - (params(0) * p_(0) + params(1)))
        .finished();
}

std::vector<Eigen::MatrixXd> ExpCurveFittingFactor::jacobians(const minisam::Variables& values) const {
    const Eigen::Vector2d& params = values.at<Eigen::Vector2d>(keys()[0]);
    return std::vector<Eigen::MatrixXd>{
        (Eigen::MatrixXd(1, 2) <<
            -params(0),
            1)
        .finished()};
}


class UsydDataPCCollectPlayBack {

    public:
        UsydDataPCCollectPlayBack(std::string& _out_file):
            out_file(_out_file),
            it(nh)
        {
            bag.open(out_file, rosbag::bagmode::Write);
            ROS_INFO_STREAM("Making output bag file: " << out_file);

            std::vector<std::string> topics{"/map", "/odom", "/tf", "/ublox_gps/fix", 
                "/tf_static", "/camera/camera_info", "/camera/rgb", "/velodyne/points"};

            ros::Time time = ros::Time::now();

            for (int i = 0; i < topics.size(); i++) {
                std::string topic = topics[i];
                TimingInfoPtr timing_info = std::make_shared<TimingInfo>();

                topic_timing_map.emplace(topic, timing_info);
            }

            undistorted_image_pub = it.advertise("usyd_dataset_pc/undistorted", 20);
            undistorted_image_lidar_pub = it.advertise("usyd_dataset_pc/undistorted_lidar", 20);
            distorted_image_pub = it.advertise("usyd_dataset_pc/distorted", 20);
            mono_before = it.advertise("usyd_dataset_pc/mono_before", 20);
            mono_after = it.advertise("usyd_dataset_pc/mono_after", 20);

            midas_depth  = std::make_shared<midas_ros::MidasDepthInterface>(nh);
            // sceneflow = std::make_shared<flow_net::FlowNetInterface>(nh);
            // mask_rcnn_interface = std::make_shared<mask_rcnn::MaskRcnnInterface>(nh);
            midas_depth->start_service();
            // sceneflow->start_service();
            // mask_rcnn_interface->start_service();
            init_distortion_params();
            loss = minisam::CauchyLoss::Cauchy(1.0);


        }

        ~UsydDataPCCollectPlayBack() {
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
            ros::Time save_time = get_timing("/ublox_gps/fix", msg->header.stamp);
            bag.write("/gps",save_time, *msg);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
            ros::Time save_time = get_timing("/velodyne/points", pc_msg->header.stamp);
            bag.write("/velodyne/points", save_time, *pc_msg);
        }

        void image_projected_points_callback(ImageConst& rgb_msg, const lidar_camera_projection::ImagePixelDepthConstPtr& projected_pts) {
            ROS_INFO_STREAM(rgb_msg->header.stamp);
            ros::Time save_time = get_timing("/camera/rgb", rgb_msg->header.stamp);
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat rgb_image = cv_ptr->image;
            std::vector<cv::Point2f> distorted_lidar_points;


            std_msgs::Header header = std_msgs::Header();
            header.stamp = save_time;

            sensor_msgs::ImagePtr  img_rgb_copy_msg = cv_bridge::CvImage(header, "rgb8", rgb_image).toImageMsg();
            distorted_image_pub.publish(img_rgb_copy_msg);


            undistort_depth_image(rgb_image, rgb_image);
            std::vector<lidar_camera_projection::PixelDepth> filtered_pixel_depth;

        
            for(const lidar_camera_projection::PixelDepth& pixel_depth : projected_pts->data) {
                double dis = pow(pixel_depth.x * pixel_depth.x + pixel_depth.y * pixel_depth.y + pixel_depth.z * pixel_depth.z, 0.5);
                int range = std::min(float(round((dis / 50) * 149)), (float) 149.0);
                
                if (range > 20) {
                    continue;
                }

                cv::Point p;
                p.x = static_cast<float>(pixel_depth.pixel_x);
                p.y = static_cast<float>(pixel_depth.pixel_y);

                distorted_lidar_points.push_back(p);
                filtered_pixel_depth.push_back(pixel_depth);


            }
            std::vector<cv::Point2f> correct_lidar_points;
            undistort_points(distorted_lidar_points,correct_lidar_points);

            std::vector<lidar_camera_projection::PixelDepth> corrected_projections;

            cv::Mat disp;
            bool mono_depth_success = midas_depth->analyse(rgb_image, disp);

            if (!mono_depth_success) {
                return;
            }

            cv::Mat rgb_lidar;
            rgb_image.copyTo(rgb_lidar);


            // disp.convertTo(disp, CV_16SC1);
            // ROS_INFO_STREAM("No points " << correct_lidar_points.size());
            // minisam::FactorGraph factor_graph;

            std::vector<double> A_vector;
            std::vector<double> b_vector;
            // typedef Eigen::Matrix<double, Eigen::Dynamic, 2> CoordMatrix;
            // typedef Eigen::Matrix<double, Eigen::Dynamic, 1> ColMatrix;
            // CoordMatrix A(correct_lidar_points.size()*2, 2);
            // ROS_INFO_STREAM(A.rows() << " " << A.cols());
            // ColMatrix b(correct_lidar_points.size()*2, 1);
            static constexpr double max_distance = 70;
            static constexpr double max_disp = 65536;
            double scaling_factor = max_disp/max_distance;

            lidar_camera_projection::ImagePixelDepth image_pixel_depth;
            image_pixel_depth.header.stamp = save_time;
            // ROS_INFO_STREAM(projected_pts->header.frame_id);

            int count = 0;
            for(int i = 0; i < correct_lidar_points.size(); i++) {
                // if (i%25 != 0) {
                //     continue;
                // }
                cv::Point p = correct_lidar_points[i];
                lidar_camera_projection::PixelDepth pixel_depth = filtered_pixel_depth[i];

                double dis = pow(pixel_depth.x * pixel_depth.x + pixel_depth.y * pixel_depth.y + pixel_depth.z * pixel_depth.z, 0.5);
                int range= std::min(float(round((dis / 50) * 149)), (float) 149.0);
                double range_d = static_cast<double>(range);


                lidar_camera_projection::PixelDepth correct_pixel;
                correct_pixel.pixel_x = p.x;
                correct_pixel.pixel_y = p.y;

                if (correct_pixel.pixel_y >= rgb_image.rows || correct_pixel.pixel_x >= rgb_image.cols || correct_pixel.pixel_x < 0 || correct_pixel.pixel_y < 0) {
                    continue;
                }
                correct_pixel.x = pixel_depth.x;
                correct_pixel.y = pixel_depth.y;
                correct_pixel.z = pixel_depth.z;

                corrected_projections.push_back(correct_pixel);
                image_pixel_depth.data.push_back(correct_pixel);

                // ROS_INFO_STREAM(pixel_depth.x);
                // ROS_INFO_STREAM(pixel_depth.y);
                // ROS_INFO_STREAM(pixel_depth.z);

                cv::circle(rgb_lidar,
                       cv::Point(correct_pixel.pixel_x, correct_pixel.pixel_y), 3,
                       CV_RGB(255 * colmap[range][0], 255 * colmap[range][1], 255 * colmap[range][2]), -1);
                double disp_map_range = (double)disp.at<uint16_t>(correct_pixel.pixel_y, correct_pixel.pixel_x)/scaling_factor;
                A_vector.push_back(disp_map_range);
                // ROS_INFO_STREAM(disp_map_range);
                // A_vector.push_back(1.0);

                //shoudl be z once looking at gmsl_center_link frame
                b_vector.push_back(static_cast<double>(pixel_depth.z));
            }



            minisam::DenseCholeskySolver cholesky_solver;
            // minisam::ConjugateGradientSolver cg;
            const int size = corrected_projections.size();
            // Eigen::Matrix<double, -1, 2> A(A_vector.data());
            // Eigen::Matrix<double, -1, 1> b(b_vector.data());
            // Eigen::Map<Eigen::MatrixXd> A(A_vector.data(), size, 1);
            // Eigen::Map<Eigen::MatrixXd> b(b_vector.data(), size, 1);
            Eigen::MatrixXd A;
            Eigen::MatrixXd b;

            A.resize(size, 2);
            b.resize(size, 1);

            for(int i = 0; i < size; i++) {
                A(i, 0) = A_vector[i];
                A(i, 1) = 1;
                b(i, 0) = b_vector[i];
            }

            Eigen::VectorXd x; //will be s, t
            x.resize(2, 1);

            // A.resize(size, 2);
            // b.resize(size, 1);
            
            // b.resize(corrected_projections.size(), 1);
            const Eigen::MatrixXd A1(A);
            const Eigen::MatrixXd b1(b);

            // cg.initialize(A1.transpose() * A1);

            minisam::LinearSolverStatus result = cholesky_solver.solve(A1.transpose() * A1, A1.transpose() * b1, x);

            if (result == minisam::LinearSolverStatus::SUCCESS) {
                // ROS_INFO_STREAM("Chol was success");
                previous_s_param =  x(0, 0);
                previous_t_param =  x(1, 0);
                ROS_INFO_STREAM("x rows:" << x.rows() << " cols: " << x.cols());
                ROS_INFO_STREAM("previous_s_param :" << previous_s_param << " previous_t_param: " << previous_t_param);


            }
            if(result == minisam::LinearSolverStatus::RANK_DEFICIENCY) {
                ROS_INFO_STREAM("Chol was RANK_DEFICIENCY");

            }
            if(result == minisam::LinearSolverStatus::INVALID) {
                ROS_INFO_STREAM("Chol was INVALID");
            }


            bag.write("/lidar_camera_projections",save_time, image_pixel_depth);



            cv::imshow("Disp before rectification", disp);
            cv::waitKey(1);
            sensor_msgs::ImagePtr  mono_before_msg = cv_bridge::CvImage(header, "mono16", disp).toImageMsg();
            bag.write("/camera/depth/unrectified_disp",save_time, mono_before_msg);
            mono_before.publish(mono_before_msg);

            sensor_msgs::ImagePtr rectified_rgb_msg = cv_bridge::CvImage(header, "bgr8", rgb_image).toImageMsg();
            bag.write("/camera/rgb/image_raw",save_time, rectified_rgb_msg);
            undistorted_image_pub.publish(rectified_rgb_msg);

            sensor_msgs::ImagePtr rectified_rgb_lidar_msg = cv_bridge::CvImage(header, "bgr8", rgb_lidar).toImageMsg();
            undistorted_image_lidar_pub.publish(rectified_rgb_lidar_msg);


            // cv::Mat disp_rectified = (previous_s_param * disp + previous_t_param);
            // cv::Mat disp_rectified = cv::Mat(disp.size(), CV_32FC1);
             cv::Mat disp_rectified;
            disp.copyTo(disp_rectified);
            for(int i = 0; i < disp.rows; i++) {
                for(int j = 0; j < disp.cols; j++) {
                    float unrectified_value = static_cast<float>(disp.at<uint16_t>(i,j));
                    float value = previous_s_param * unrectified_value/scaling_factor + previous_t_param;
                    // ROS_INFO_STREAM(unrectified_value << " " << value << " " <<  value * scaling_factor);
                    disp_rectified.at<uint16_t>(i, j) = static_cast<uint16_t>(value * scaling_factor);
                    // uint16_t new_vale = value * scaling_factor;
                    // ROS_INFO_STREAM(disp_rectified.at<uint16_t>(i, j));
                    //  ROS_INFO_STREAM(new_vale);
                    // disp_rectified.at<uint16_t>(i, j) = new_vale;
                    //  ROS_INFO_STREAM(disp_rectified.at<uint16_t>(i, j));
                    // double new_value = previous_s_param * static_cast<double>(disp.at<uint16_t>(i,j)/scaling_factor) + previous_t_param;
                    // uint16_t scaled = static_cast<uint16_t>(new_value * scaling_factor);
                    // disp_rectified.at<uint16_t>(i, j) = scaled;
                }
            }


            // cv::bitwise_not(disp_rectified, disp_rectified);



            cv::imshow("Projection", rgb_image);
            cv::waitKey(1);
            sensor_msgs::ImagePtr  img_msg = cv_bridge::CvImage(header, "mono16", disp_rectified).toImageMsg();
            bag.write("/camera/depth/image_raw",save_time, *img_msg);
            mono_after.publish(img_msg);
            cv::imshow("Disp after dectification", disp_rectified);
            cv::waitKey(1);
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

        typedef std::shared_ptr<UsydDataPCCollectPlayBack::TimingInfo> TimingInfoPtr;

        midas_ros::MidasDepthInterfacePtr midas_depth;


        std::string out_file;
        rosbag::Bag bag;
        ros::NodeHandle nh;
        CameraInformation camera_info;

        image_transport::ImageTransport it;
        image_transport::Publisher undistorted_image_pub;
        image_transport::Publisher undistorted_image_lidar_pub;
        image_transport::Publisher distorted_image_pub;
        image_transport::Publisher mono_before;
        image_transport::Publisher mono_after;

        std::shared_ptr<minisam::LossFunction> loss;
        
        void init_distortion_params();
        void undistort_depth_image(const cv::Mat& depth, cv::Mat& depth_undistorted);
        // void undistort_lidar_points(const cv::Mat& points, cv::Mat& undistorted_points);
        void undistort_points(const std::vector<cv::Point2f>& distorted_points, std::vector<cv::Point2f>& undistorted_points);
    

        std::map<std::string, UsydDataPCCollectPlayBack::TimingInfoPtr> topic_timing_map;

        ros::Time update_timing(const std::string& topic, const ros::Time& msg_time);
        double previous_t_param = 1;
        double previous_s_param = 1;

};

void UsydDataPCCollectPlayBack::init_distortion_params() {
    std::string input_camera_info_topic = "/gmsl/A0/camera_info";
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

    // std::string input_video_topic = "/vdoslam/input/camera/rgb/image_raw";
    // std::string camera_info_topic = video_stream_namespace + "camera_info";
    // std::string mask_rcnn_topic = "/vdoslam/input/camera/mask/image_raw";
    // std::string flow_net_topic = "/vdoslam/input/camera/flow/image_raw";
    // std::string monodepth_topic = "/vdoslam/input/camera/depth/image_raw";


    // //topics for viz
    // std::string mask_rcnn_viz_topic = "/vdoslam/input/camera/mask/colour_mask";
    // std::string flow_net_viz_topic = "/vdoslam/input/camera/flow/colour_map";

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
    ros::Subscriber sub_odom = n.subscribe(odom_topic, 100, &UsydDataPCCollectPlayBack::odom_callback, &playback);
    ros::Subscriber sub_map = n.subscribe(map_topic, 100, &UsydDataPCCollectPlayBack::map_callback, &playback);
    ros::Subscriber sub_tf_static = n.subscribe(tf_static_topic, 100, &UsydDataPCCollectPlayBack::tf_static_callback, &playback);
    ros::Subscriber sub_tf = n.subscribe(tf_topic, 100, &UsydDataPCCollectPlayBack::tf_callback, &playback);
    ros::Subscriber sub_camera_info = n.subscribe("/gmsl/A0/camera_info", 100, &UsydDataPCCollectPlayBack::camera_info_callback, &playback);
    ros::Subscriber gps_sub_info = n.subscribe(gps_topic, 100, &UsydDataPCCollectPlayBack::gps_info_callback, &playback);

    ros::Subscriber sub_lidar_camera_projection = n.subscribe("/velodyne/front/corrected", 100, &UsydDataPCCollectPlayBack::pc_callback, &playback);

    // //these ones we must synchronize to a single time
    message_filters::Subscriber<sensor_msgs::Image> image_raw_sub(n, "/gmsl/A0/image_color", 10);
    message_filters::Subscriber<lidar_camera_projection::ImagePixelDepth> pixel_depth(n, "/gmsl/A0/image_color/image_pixel_depth", 10);
    // message_filters::Subscriber<sensor_msgs::Image> mask_rcnn_viz_sub(n, mask_rcnn_viz_topic, 10);

    // message_filters::Subscriber<sensor_msgs::Image> flow_sub(n, flow_net_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> flow_viz_sub(n, flow_net_viz_topic, 10);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, monodepth_topic, 10);

    message_filters::TimeSynchronizer<sensor_msgs::Image, lidar_camera_projection::ImagePixelDepth> sync(image_raw_sub,
                                                                                                       pixel_depth,  1000);


    ///velodyne/front/corrected

    sync.registerCallback(boost::bind(&UsydDataPCCollectPlayBack::image_projected_points_callback, &playback, _1, _2));
    // ros::Subscriber vdo_input_sub = n.subscribe("/vdoslam/input/all",100, &UsydDataPCCollectPlayBack::vdo_input_callback, &playback );

    ros::spin();
    // playback.close();

}