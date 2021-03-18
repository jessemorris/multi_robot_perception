#include "utils/RosUtils.hpp"

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Transform.h>

namespace VDO_SLAM {

    namespace utils {

        void mat_to_image_msg(sensor_msgs::Image& img_msg, const cv::Mat& img, const std::string& encoding, const std_msgs::Header& header) {
            cv_bridge::CvImage img_ptr = cv_bridge::CvImage(header, encoding, img);
            img_ptr.toImageMsg(img_msg);
        }

        void image_msg_to_mat(cv::Mat& img, const sensor_msgs::Image& image_msg, const std::string& encoding) {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, encoding);
                img = cv_ptr->image;
        }

        void publish_static_tf(const nav_msgs::Odometry& odom,
                                const std::string& parent_frame_id,
                                const std::string& child_frame_id, const ros::Time& time) {
                                    
            static tf2_ros::StaticTransformBroadcaster static_broadcaster;
            geometry_msgs::TransformStamped static_transform_stamped;
            // TODO(Toni): Warning: using ros::Time::now(), will that bring issues?
            static_transform_stamped.header.stamp = ros::Time::now();
            static_transform_stamped.header.frame_id = parent_frame_id;
            static_transform_stamped.child_frame_id = child_frame_id;

            
            odom_to_tf(odom, &static_transform_stamped.transform);
            static_broadcaster.sendTransform(static_transform_stamped);
            ros::spinOnce();
        }

        void odom_to_tf(const nav_msgs::Odometry& odom, geometry_msgs::Transform* transform) {
            transform->translation.x = odom.pose.pose.position.x;
            transform->translation.y = odom.pose.pose.position.y;
            transform->translation.z = odom.pose.pose.position.z;

            transform->rotation.x = odom.pose.pose.orientation.x;
            transform->rotation.y = odom.pose.pose.orientation.y;
            transform->rotation.z = odom.pose.pose.orientation.z;
            transform->rotation.w = odom.pose.pose.orientation.w;

        }

        void geometry_to_odom(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const ros::Time& time, nav_msgs::Odometry& odom) {
            odom.header.stamp = time;
            odom.pose.pose = pose;
            odom.twist.twist = twist;
        }

        void geometry_to_odom(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const ros::Time& time, 
                                        const std::string& frame_id, const std::string& child_frame_id, nav_msgs::Odometry& odom) {
            odom.header.stamp = time;
            odom.header.frame_id = frame_id;
            odom.child_frame_id = child_frame_id;
            odom.pose.pose = pose;
            odom.twist.twist = twist;
                                    
        }

    }
}