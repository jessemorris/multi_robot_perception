#ifndef _ROS_VDO_IMAGE_PREPROCESSING_RGB
#define _ROS_VDO_IMAGE_PREPROCESSING_RGB

#include "ImagePreprocessing.hpp"

namespace VDO_SLAM {

    namespace preprocessing {

        class ImageRGB : public BaseProcessing {

                public:
                    ImageRGB(ros::NodeHandle& nh_);

                    void image_callback(ImageConstPtr& rgb);
                    void start_services() override;

                    
        };

    }

};

#endif