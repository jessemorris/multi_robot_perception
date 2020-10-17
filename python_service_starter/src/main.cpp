#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>

#include "PythonServiceStarter.hpp"



int main(int argc, char ** argv) {

    ros::init(argc, argv, "python_service_starter");

    PythonServiceStarter python_service_starter;
    python_service_starter.init_flow_net();
    python_service_starter.init_mask_rcnn();
    ros::spin();

}