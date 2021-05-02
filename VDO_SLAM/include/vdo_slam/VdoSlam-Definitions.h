#ifndef VDO_SLAM_DEFINITIONS_H
#define VDO_SLAM_DEFINITIONS_H

    #ifdef __VDO_SLAM_DIR__
        constexpr std::string VDO_SLAM_DIR = __VDO_SLAM_DIR__;
        std::cout << VDO_SLAM_DIR << std::endl;
    #elif
        constexpr std::string VDO_SLAM_DIR = "../";
    #endif


#endif