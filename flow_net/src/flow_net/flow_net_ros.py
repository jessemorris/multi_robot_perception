import time
import os
import struct
import rospy
import numpy as np


from path import Path

import torch
import torch.backends.cudnn as cudnn
import torch.nn.functional as F
import torchvision.transforms as transforms

from imageio import imread, imwrite
from src.flow_net import python_models


from src.flow_net.utils import flow_transforms
from  src.flow_net.utils.util import flow2rgb
from flow_net.srv import FlowNet

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#TODO relative paths
package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/flow_net/"


class FlowNetRos():

    def __init__(self, model_path= package_path + "src/flow_net/models/flownetc_EPE1.766.tar"):
        self.model_path = model_path
        self._write_fd = int(os.getenv("flow_net_PY_WRITE_FD"))
        self.write_pipe = os.fdopen(self._write_fd, 'wb', 0)


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
        self.log_to_ros("Service call ready")

    def log_to_ros(self, msg):
        msg_size = struct.pack('<I', len(msg))
        self.write_pipe.write(msg_size)
        self.write_pipe.write(msg.encode("utf-8"))


    def flow_net_service_callback(self, req):
        pass