import time
import os
import struct
import rospy
import numpy as np
import ros_numpy
import math

from path import Path

import torch
import torch.backends.cudnn as cudnn
import torch.nn.functional as F
import torchvision.transforms as transforms

from imageio import imread, imwrite



from src.flow_net.layers import Network

from flow_net.srv import FlowNet, FlowNetResponse
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator


import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

#TODO relative paths
package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/flow_net/"


class FlowNetRos(RosCppCommunicator):

    def __init__(self, model_path= package_path + "src/flow_net/models/network-default.pytorch"):
        RosCppCommunicator.__init__(self)
        self.model_path = model_path
       
        torch.set_grad_enabled(False)  # make sure to not compute gradients for computational performance
        cudnn.enabled = True # make sure to use cudnn for computational performance

        self.network = Network(self.model_path).cuda().eval()

        #set up service calls
        self.flow_net_service = rospy.Service("flow_net_service",FlowNet, self.flow_net_service_callback)
        self.flow_net_test_publisher = rospy.Publisher('flow_net/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")


    @torch.no_grad()
    def flow_net_service_callback(self, req):
        response = FlowNetResponse()
        try:
            previous_image = ros_numpy.numpify(req.previous_image)
            current_image = ros_numpy.numpify(req.current_image)

            image = ros_numpy.msgify(Image, previous_image, encoding='rgb8')
            image.header.frame_id = "base_link"
            self.flow_net_test_publisher.publish(image)

            # previous_image = self.bridge.imgmsg_to_cv2(req.previous_image, "bgr8")
            # current_image = self.bridge.imgmsg_to_cv2(req.current_image, "bgr8")
        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response


        output_tensor = self.analyse_flow(previous_image, current_image)

        self.log_to_ros(output_tensor.shape)

        # rgb_flow = self.flow2rgb(output_tensor)
        # output_image = (rgb_flow * 255).astype(np.uint8).transpose(1,2,0)
        output_image = output_tensor.numpy().astype(np.float32).transpose(1,2,0)
        print(output_image.shape)

        # output_image_msg = ros_numpy.msgify(Image, output_image, encoding='rgb8')
        output_image_msg = ros_numpy.msgify(Image, output_image, encoding='32FC2')
        response.success = True
        response.output_image = output_image_msg

        return response

    #inputs should be a numpy array
    #returns the output of the nerual network as a tensor 2 x N x M tensor
    def analyse_flow(self, previous_image, current_image):

        #convert to tensor array
        tenFirst = torch.FloatTensor(np.ascontiguousarray(np.array(previous_image)[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) * (1.0 / 255.0)))
        tenSecond = torch.FloatTensor(np.ascontiguousarray(np.array(current_image)[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) * (1.0 / 255.0)))

        assert(tenFirst.shape[1] == tenSecond.shape[1])
        assert(tenFirst.shape[2] == tenSecond.shape[2])

        intWidth = tenFirst.shape[2]
        intHeight = tenFirst.shape[1]

        # assert(intWidth == 1024) # remember that there is no guarantee for correctness, comment this line out if you acknowledge this and want to continue
        # assert(intHeight == 436) # remember that there is no guarantee for correctness, comment this line out if you acknowledge this and want to continue

        tenPreprocessedFirst = tenFirst.cuda().view(1, 3, intHeight, intWidth)
        tenPreprocessedSecond = tenSecond.cuda().view(1, 3, intHeight, intWidth)

        intPreprocessedWidth = int(math.floor(math.ceil(intWidth / 32.0) * 32.0))
        intPreprocessedHeight = int(math.floor(math.ceil(intHeight / 32.0) * 32.0))

        tenPreprocessedFirst = torch.nn.functional.interpolate(input=tenPreprocessedFirst, size=(intPreprocessedHeight, intPreprocessedWidth), mode='bilinear', align_corners=False)
        tenPreprocessedSecond = torch.nn.functional.interpolate(input=tenPreprocessedSecond, size=(intPreprocessedHeight, intPreprocessedWidth), mode='bilinear', align_corners=False)

        tenFlow = torch.nn.functional.interpolate(input=self.network(tenPreprocessedFirst, tenPreprocessedSecond), size=(intHeight, intWidth), mode='bilinear', align_corners=False)

        tenFlow[:, 0, :, :] *= float(intWidth) / float(intPreprocessedWidth)
        tenFlow[:, 1, :, :] *= float(intHeight) / float(intPreprocessedHeight)

        flow = tenFlow[0, :, :, :].cpu()

        del tenFlow
        del tenPreprocessedFirst
        del tenPreprocessedSecond
        del tenFirst
        del tenSecond
        
        return flow

    def flow2rgb(self, flow_map):
        flow_map = flow_map.squeeze(0)
        flow_map_np = flow_map.detach().cpu().numpy()
        _, h, w = flow_map_np.shape
        flow_map_np[:,(flow_map_np[0] == 0) & (flow_map_np[1] == 0)] = float('nan')

        rgb_map = np.ones((3,h,w)).astype(np.float32)
        # if max_value is not None:
        #     normalized_flow_map = flow_map_np / max_value
        # else:
        normalized_flow_map = flow_map_np / (np.abs(flow_map_np).max())
        rgb_map[0] += normalized_flow_map[0]
        rgb_map[1] -= 0.5*(normalized_flow_map[0] + normalized_flow_map[1])
        rgb_map[2] += normalized_flow_map[1]
        return rgb_map.clip(0,1)



