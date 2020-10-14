#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>

#include "PythonServiceStarter.hpp"



int main(int argc, char ** argv) {

    ros::init(argc, argv, "python_service_starter");

    PythonServiceStarter python_service_starter;
    ros::spin();

}