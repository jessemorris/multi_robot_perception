#ifndef _ROS_VDO_SCENE_GRAPHS_ONLINE_PROVIDER
#define _ROS_VDO_SCENE_GRAPHS_ONLINE_PROVIDER

#include <ros/ros.h>
#include <vector>

#include "DataProviderInterface.hpp"

namespace VDO_SLAM {


    class RosOnlineDataProvider : public DataProviderInterface {

        public:
            RosOnlineDataProvider();

            bool spin() override;

        private:

            std::string input_topic;

    };


}

#endif