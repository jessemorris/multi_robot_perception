#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>
#include <memory>

#include "python_service_starter/StartFlowNet.h"
#include "PipeCommsManager.hpp"


class PythonServiceStarter {

    public:
        PythonServiceStarter();
        ~PythonServiceStarter();

        // bool init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response);
        bool init_flow_net();
        //TODO
        // bool shutdown_services();
  
    private:
        bool flow_status;
        // PipeCommsManager flow_net_communication;

        bool run_output_threads;

        ros::NodeHandle nh;
        // ros::ServiceServer start_flow_net_service;

        std::string python_flow_net_full_path;

        std::unique_ptr<PipeCommsManager> pipe_comms_manager;



};
