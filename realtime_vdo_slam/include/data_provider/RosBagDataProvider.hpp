#ifndef _ROS_VDO_SCENE_GRAPHS_BAG_DATA_PROVIDER
#define _ROS_VDO_SCENE_GRAPHS_BAG_DATA_PROVIDER

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <vector>
#include <memory>

#include "DataProviderInterface.hpp"

namespace VDO_SLAM {


    class RosBagDataProvider : public DataProviderInterface {

        public:
            RosBagDataProvider();

            bool spin() override;

        private:
            std::string scene_topic, camera_info_topic;
            std::vector<std::string> topics;

            
            std::string bag_file_name;
            rosbag::Bag bag;

    };

    typedef std::shared_ptr<RosBagDataProvider> RosBagDataProviderPtr;
    typedef std::unique_ptr<RosBagDataProvider> RosBagDataProviderUniquePtr;


}

#endif