#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>
#include <memory>

#include "PythonServiceStarter.hpp"



PythonServiceStarter::PythonServiceStarter() :
    nh("~"),
    flow_status(false),
    run_output_threads(true),
    python_flow_net_full_path("/home/jesse/Code/src/ros/src/multi_robot_perception/flow_net/scripts/flow_net_interface.py")
{
    // start_flow_net_service = nh.advertiseService("start_flow_net", &PythonServiceStarter::init_flow_net, this);
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

// bool PythonServiceStarter::init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response) {
bool PythonServiceStarter::init_flow_net() {
    // if (request.start) {


        int pipe_py_to_cpp[2];

        if (::pipe(pipe_py_to_cpp)) {
            ROS_WARN_STREAM("Could not open pipes");
            return false;
        }
        else {
            pid_t pid = fork();

            if (pid == 0) {
                ROS_INFO_STREAM("Setting up env variables for program flow_net.py" );
                ::close(pipe_py_to_cpp[0]);

                std::string program_name("flow_net");
                std::ostringstream oss;

                ROS_INFO_STREAM(pipe_py_to_cpp[1]);

                // oss << "export " << program_name << "_ " << "PY_WRITE_FD=" << pipe_py_to_cpp[1] << " && "
                //     << "export PYTHONUNBUFFERED=true && " // Force stdin, stdout and stderr to be totally unbuffered.
                //     << "python3 " << python_flow_net_full_path;
                setenv(std::string(program_name + "_" + "PY_WRITE_FD").c_str(), std::to_string(pipe_py_to_cpp[1]).c_str(), 1);
                setenv("PYTHONUNBUFFERED", "true", 1);
                oss << "python3 " << python_flow_net_full_path;


                ::system(oss.str().c_str());
                ::close(pipe_py_to_cpp[1]);
                flow_status = true;
                ros::Duration(3).sleep();
            }
            else if (pid < 0) {
                ROS_WARN_STREAM("Program forking failed");
                return false;
            }
            else {

                pipe_comms_manager = std::make_unique<PipeCommsManager>("flow_net", pipe_py_to_cpp, pid);
                ::close(pipe_py_to_cpp[1]);

                pipe_comms_manager->run_listener();
                

            }

                
        }

    // }
    return true;
}


