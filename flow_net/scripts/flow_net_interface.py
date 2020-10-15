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

# time.sleep(3)
# _w_fd = int(os.getenv("flow_net_PY_WRITE_FD"))


# _w_pipe = os.fdopen(_w_fd, 'wb', 0)

# def write(msg):
#     msg_size = struct.pack('<I', len(msg))
#     _w_pipe.write(msg_size)
#     _w_pipe.write(msg.encode("utf-8"))



# while 1:
#     write("look a new message")
#     time.sleep(2)



# @torch.no_grad()
# def main():
    
#     # Data loading code
#     input_transform = transforms.Compose([
#         flow_transforms.ArrayToTensor(),
#         transforms.Normalize(mean=[0,0,0], std=[255,255,255]),
#         transforms.Normalize(mean=[0.411,0.432,0.45], std=[1,1,1])
#     ])

#     pretrained_model = "pretrained/flownetc_EPE1.766.tar"
#     img1_file = "images/kitti1.png"
#     img2_file = "images/kitti2.png"
#     # create model
#     network_data = torch.load(pretrained_model)
#     print(network_data)
#     print("=> using pre-trained model '{}'".format(network_data['arch']))
#     model = models.__dict__[network_data['arch']](network_data).to(device)
#     model.eval()

#     cudnn.benchmark = True

#     #default
#     div_flow = 20
#     max_flow = None

#     if 'div_flow' in network_data.keys():
#         print("flow is {}".format(network_data['div_flow']))
#         div_flow = network_data['div_flow']

#     print("Image 1 file {}".format(img1_file))
#     print("Image 2 file {}".format(img2_file))


#     img1 = input_transform(imread(img1_file))
#     print(type(img1))
#     print(img1.shape)
#     img2 = input_transform(imread(img2_file))
#     input_var = torch.cat([img1, img2]).unsqueeze(0)


#     # if args.bidirectional:
#     #     # feed inverted pair along with normal pair
#     #     inverted_input_var = torch.cat([img2, img1]).unsqueeze(0)
#     #     input_var = torch.cat([input_var, inverted_input_var])

#     input_var = input_var.to(device)
#     # compute output
#     output = model(input_var)
#     print(output.shape)
#     # if args.upsampling is not None:
#     #     output = F.interpolate(output, size=img1.size()[-2:], mode=args.upsampling, align_corners=False)
#     for suffix, flow_output in zip(['flow', 'inv_flow'], output):
#         filename = "images/output"
#         rgb_flow = flow2rgb(div_flow * output, max_value=max_flow)
#         to_save = (rgb_flow * 255).astype(np.uint8).transpose(1,2,0)
#         imwrite(filename + '.png', to_save)


if __name__ == '__main__':
    rospy.init_node('flow_net_interface')
    flow_net_ros = FlowNetRos()
    rospy.spin()