#ifndef _ROS_VDO_IMAGE_PREPROCESSING_RGBD
#define _ROS_VDO_IMAGE_PREPROCESSING_RGBD

#include "ImagePreprocessing.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include "CameraInformation.hpp"

typedef const sensor_msgs::ImageConstPtr& ImageConst;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;


namespace VDO_SLAM {

    namespace preprocessing {

        class ImageRgbDepth : public BaseProcessing {

            public:

                ImageRgbDepth(ros::NodeHandle& nh_);
                void image_callback(ImageConst raw_image, ImageConst depth);

                void start_services() override;

            private:
                message_filters::Subscriber<sensor_msgs::Image> raw_img_synch;
                message_filters::Subscriber<sensor_msgs::Image> depth_img_synch;

                // message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;

                message_filters::Synchronizer<MySyncPolicy> sync;

                    
        };

    }

};

#endif