import time
import os
import struct
import rospy
import numpy as np
import ros_numpy

from path import Path

import torch
import torch.backends.cudnn as cudnn
import torch.nn.functional as F
import torchvision.transforms as transforms

from imageio import imread, imwrite
from src.flow_net import python_models


from src.flow_net.utils import flow_transforms
from src.flow_net.utils.util import flow2rgb
from flow_net.srv import FlowNet, FlowNetResponse

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

#TODO relative paths
package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/flow_net/"


class FlowNetRos():

    def __init__(self, model_path= package_path + "src/flow_net/models/flownetc_EPE1.766.tar"):
        self.model_path = model_path
        self._write_fd = int(os.getenv("flow_net_PY_WRITE_FD"))
        self.write_pipe = os.fdopen(self._write_fd, 'wb', 0)

        # self.bridge = CvBridge()


        if self.write_pipe == None:
            #do stuff
            pass

        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.log_to_ros("Devis is {}".format(self.device))

        self.input_transform = transforms.Compose([
            flow_transforms.ArrayToTensor(),
            transforms.Normalize(mean=[0,0,0], std=[255,255,255]),
            transforms.Normalize(mean=[0.411,0.432,0.45], std=[1,1,1])
        ])

        self.network_data = torch.load(self.model_path)
        # print(self.network_data)
        self.log_to_ros("=> using pre-trained model '{}'".format(self.network_data['arch']))
        self.model = python_models.__dict__[self.network_data['arch']](self.network_data).to(self.device)
        self.model.eval()

        cudnn.benchmark = True

        #params
        #default
        self.div_flow = 20
        self.max_flow = None

        if 'div_flow' in self.network_data.keys():
            self.log_to_ros("flow is {}".format(self.network_data['div_flow']))
            self.div_flow = self.network_data['div_flow']

        #set up service calls
        self.flow_net_service = rospy.Service("flow_net_service",FlowNet, self.flow_net_service_callback)
        self.flow_net_test_publisher = rospy.Publisher('flow_net/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")

    def log_to_ros(self, msg):
        msg_size = struct.pack('<I', len(msg))
        self.write_pipe.write(msg_size)
        self.write_pipe.write(msg.encode("utf-8"))


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


        previous_image_tensor = self.input_transform(previous_image)
        current_image_tensor = self.input_transform(current_image)
        input_tensor = torch.cat([previous_image_tensor, current_image_tensor]).unsqueeze(0)

        input_tensor = input_tensor.to(self.device)
        # compute output
        output_tensor = self.model(input_tensor)

        #choices=['nearest', 'bilinear']
        output_tensor = F.interpolate(output_tensor, size=previous_image_tensor.size()[-2:], mode='bilinear', align_corners=False)
        rgb_flow = flow2rgb(self.div_flow * output_tensor, max_value=self.max_flow)
        output_image = (rgb_flow * 255).astype(np.uint8).transpose(1,2,0)

        # print(output_image.shape)

        # bgr_flow = flow2bgr(self.div_flow * output_tensor, max_value=self.max_flow)
        # output_image = (bgr_flow * 255).astype(np.uint8).transpose(1,2,0)

        # self.log_to_ros(str(output_image.shape))

        #note this is RGB!
        # output_image_msg = self.bridge.cv2_to_imgmsg(output_image, "rgb8")

        output_image_msg = ros_numpy.msgify(Image, output_image, encoding='rgb8')
        response.success = True
        response.output_image = output_image_msg

        return response
