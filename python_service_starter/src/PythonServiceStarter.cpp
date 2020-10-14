#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>

#include "PythonServiceStarter.hpp"



PythonServiceStarter::PythonServiceStarter() :
    nh("~"),
    flow_status(false),
    run_output_threads(true),
    python_flow_net_full_path("/home/jesse/Code/src/ros/src/multi_robot_perception/FlowNetPytorch/")
{
    start_flow_net_service = nh.advertiseService("start_flow_net", &PythonServiceStarter::init_flow_net, this);
}

PythonServiceStarter::~PythonServiceStarter() {
    // if (ik_node_status) {
    //     pclose(pipe_in_IK);
    // }

    // if (grasp_node_status) {
    //     pclose(pipe_in_grasp);
    // }

    // if (pose_cnn_node_status) {
    //     pclose(pipe_in_pose_cnn);
    // }

    // if (dae_mesh_node_status) {
    //     pclose(pipe_in_dae_mesh);
    // }
}

bool PythonServiceStarter::init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response) {

    if (request.start) {
        ROS_INFO_STREAM("Starting Python IK script");
        std::string command = "python3 " + python_flow_net_full_path;
        ROS_INFO_STREAM("executing command: " << command);
        pipe_in_IK = popen(command.c_str(), "r");
        flow_status = true;

        ros::Duration(3).sleep();

        char buff[255];
        // fscanf(pipe_in_IK, "%s", buff);
        //
        // ROS_INFO_STREAM("file output: " << std::string(buff));
    }
    return true;
}


