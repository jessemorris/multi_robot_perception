#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>

#include "python_service_starter/StartFlowNet.h"
#include "PipeCommsManager.hpp"


class PythonServiceStarter {

    public:
        PythonServiceStarter();
        ~PythonServiceStarter();

        bool init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response);
  
    private:
        bool flow_status;
        PipeCommsManager flow_net_communication;

        bool run_output_threads;

        ros::NodeHandle nh;
        ros::ServiceServer start_flow_net_service;

        std::string python_flow_net_full_path;

        FILE* pipe_in_IK;

        std::thread pose_cnn_output_thread;

        void pose_cnn_output_function();
};
