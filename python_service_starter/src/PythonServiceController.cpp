#include <ros/ros.h>
#include <string>
#include <thread>
#include <sstream>
#include <memory>

#include "PythonServiceController.hpp"



PythonServiceController::PythonServiceController(ros::NodeHandle& _nh) :
    nh(_nh),
    flow_status(false),
    mask_rcnn_status(false),
    run_output_threads(true),
    python_flow_net_full_path("/home/jesse/Code/src/ros/src/multi_robot_perception/flow_net/scripts/flow_net_interface.py"),
    python_mask_rcnn_full_path("/home/jesse/Code/src/ros/src/multi_robot_perception/mask_rcnn/scripts/mask_rcnn_interface.py")
{
    start_flow_net_service = nh.advertiseService("start_flow_net", &PythonServiceController::init_flow_net, this);
    start_mask_rcnn_service = nh.advertiseService("start_mask_rcnn", &PythonServiceController::init_mask_rcnn, this);

}

PythonServiceController::~PythonServiceController() {
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

bool PythonServiceController::init_flow_net(python_service_starter::StartFlowNet::Request& request, python_service_starter::StartFlowNet::Response& response) {
    if (request.start) {
        flow_net_service_starter = std::make_unique<ServiceStarter>(python_flow_net_full_path);
        
        if(flow_net_service_starter->start_program()) {
            ROS_INFO_STREAM("program " << flow_net_service_starter->get_program_name() << " started");
            return true;
        }
        else {
            ROS_WARN_STREAM("program " << flow_net_service_starter->get_program_name() << " failed");
            return false;
        }
        
    }
    return true;
}



bool PythonServiceController::init_mask_rcnn(python_service_starter::StartMaskRcnn::Request& request, python_service_starter::StartMaskRcnn::Response& response) {
    if (request.start) {
        mask_rcnn_service_starter = std::make_unique<ServiceStarter>(python_mask_rcnn_full_path);
        
        if(mask_rcnn_service_starter->start_program()) {
            ROS_INFO_STREAM("program " << mask_rcnn_service_starter->get_program_name() << " started");
            return true;
        }
        else {
            ROS_WARN_STREAM("program " << mask_rcnn_service_starter->get_program_name() << " failed");
            return false;
        }
        
    }
    return true;
}

bool PythonServiceController::shutdown_services() {
    if (flow_net_service_starter && flow_net_service_starter->get_status()) {
        flow_net_service_starter->shutdown();
    }
    if (mask_rcnn_service_starter && mask_rcnn_service_starter->get_status()) {
        mask_rcnn_service_starter->shutdown();
    }
    return true;
}

