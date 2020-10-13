#ifndef _REALTIME_VDO_SLAM
#define _REALTIME_VDO_SLAM


#include <ros/ros.h>
#include <nodelet/loader.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <stdio.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


#include "SceneFlow.hpp"

#include <string>
#include <vector>
#include <algorithm>
#include <map>


class RealTimeVdoSLAM {

    public:
        RealTimeVdoSLAM();
        ~RealTimeVdoSLAM();


    private:
        ros::Subscriber image_subscriber;
        ros::Publisher results;

        

};



#endif