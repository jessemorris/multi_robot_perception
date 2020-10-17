# import time
# import os
# import struct

# import argparse
# from path import Path

# import torch
# import torch.backends.cudnn as cudnn
# import torch.nn.functional as F
# import models

# import torchvision.transforms as transforms
# import utils.flow_transforms
# from imageio import imread, imwrite
# import numpy as np
# from utils.util import flow2rgb
import os
import sys
sys.path.append("/home/jesse/Code/src/ros/src/multi_robot_perception/flow_net")
from src.flow_net.flow_net_ros import FlowNetRos
import rospy



if __name__ == '__main__':
    rospy.init_node('flow_net_interface')
    flow_net_ros = FlowNetRos()
    rospy.spin()