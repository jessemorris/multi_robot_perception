#ifndef _ROS_VDO_IMAGE_PREPROCESSING_RGBD
#define _ROS_VDO_IMAGE_PREPROCESSING_RGBD

#include "ImagePreprocessingBase.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include "CameraInformation.hpp"

typedef const sensor_msgs::ImageConstPtr& ImageConst;

namespace VDO_SLAM {

    namespace preprocessing {

        class ImageRGBD : public BaseProcessing {

                public:
                    ImageRGBD(ros::NodeHandle& n);
                    ~ImageRGBD() {}

                    void image_callback(ImageConst raw_image, ImageConst depth);

                private:
                    message_filters::Subscriber<sensor_msgs::Image> raw_img;
                    message_filters::Subscriber<sensor_msgs::Image> depth_img;

                    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;

                    
        };

    }

};

#endif