#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

// #include <opencv2/features2d.hpp>
#include <memory>

#include <vdo_slam/ORBextractor.h>

/**
 * @brief Node to test the accuracy of orb exrtractor used in VDO SLAM. 
 * 
 * Is an issue becuase the feature extractor is incorrect and this test is used to try and isolate the issue
 * Subscribes to "/camera/image_raw" and "/camera/camera_info".
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */

class OrbTestBase {

    public:
        OrbTestBase(ros::NodeHandle& _nh) :
                nh(_nh),
                image_transport(_nh) {

            image_subscriber = image_transport.subscribe("/camera/rgb/image_raw", 20,
                                        &OrbTestBase::image_callback, this);

            feature_pub = image_transport.advertise("/vdoslam_tests/featuer_image", 10);

        }
        virtual void image_callback(const sensor_msgs::ImageConstPtr& msg) = 0;

    protected:


        image_transport::ImageTransport image_transport;
        image_transport::Subscriber image_subscriber;
        image_transport::Publisher feature_pub;
        ros::NodeHandle nh;
        
        
};

class CVOrbExtractorTest : public OrbTestBase {

    public:
        CVOrbExtractorTest(ros::NodeHandle& _nh) :
            OrbTestBase(_nh) {

                detector = cv::ORB::create();
                descriptor = cv::ORB::create();
                matcher = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );

        }

        void image_callback(const sensor_msgs::ImageConstPtr& msg) override {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat image = cv_ptr->image;

            detector->detect (image,keypoints);

            descriptor->compute (image, keypoints, descriptors);

            cv::Mat outimg;
            cv::drawKeypoints(image, keypoints, outimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "rgb8", outimg).toImageMsg();
            feature_pub.publish(img_msg);

        }

    private:
        
        //VDO-SLAM uses their own custom ORBextractor. We start with the basic opencv one
        cv::Ptr<cv::FeatureDetector> detector;
        cv::Ptr<cv::DescriptorExtractor> descriptor;
        cv::Ptr<cv::DescriptorMatcher> matcher;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
};

class VDOOrbExtractorTest : public OrbTestBase {

    public:
        VDOOrbExtractorTest(ros::NodeHandle& _nh) :
            OrbTestBase(_nh) {

                    //params taken exactly from vdo config.yaml file
                    int nFeatures = 2500;
                    float fScaleFactor = 1.2;
                    int nLevels = 8;
                    int fIniThFAST = 20;
                    int fMinThFAST = 7;
                    ROS_INFO_STREAM("Createding Orb extractor...");
                    orb_extractor = std::unique_ptr<VDO_SLAM::ORBextractor>(new VDO_SLAM::ORBextractor(nFeatures,
                                        fScaleFactor,nLevels,fIniThFAST,fMinThFAST));

                    ROS_INFO_STREAM("Created Orb extractor");
                
            }

        void image_callback(const sensor_msgs::ImageConstPtr& msg) override {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
            cv::Mat image = cv_ptr->image;

            (*orb_extractor)(image,cv::Mat(),mv_keys,m_descriptors);
            cv::Mat outimg;
            cv::drawKeypoints(image, mv_keys, outimg, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "rgb8", outimg).toImageMsg();
            feature_pub.publish(img_msg);
        }

    private:
        std::unique_ptr<VDO_SLAM::ORBextractor> orb_extractor;
        std::vector<cv::KeyPoint> mv_keys;

         // ORB descriptor, each row associated to a keypoint.
        cv::Mat m_descriptors;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vdo_orb_test_node");
    ros::NodeHandle n;

    // OrbTestBase test(n);

    //OPENCV ORB TEST
    CVOrbExtractorTest orb_tests(n);

    // VDOOrbExtractorTest orb_tests(n);
    ros::spin();



}