
from flow_net.flow_net_ros import FlowNetRos
from vdo_slam_tests.utils import *

import rospy
import rospkg
import cv2
import numpy as np
import sys


rospack = rospkg.RosPack()
package_path = rospack.get_path("vdo_slam_tests")



if __name__ == "__main__":
    rospy.init_node("kitti_comparison_flow")

    image_name = parse_image_name(sys.argv)
    prev_image_num = int(image_name) - 1

    assert(prev_image_num > -1)

    if prev_image_num > 9:
        prev_image_name = "0000" + str(prev_image_num)
    else:
        prev_image_name = "00000" + str(prev_image_num)

    flow_data_path = package_path + "/kitti_data/flow/" + image_name + ".flo"
    current_data_path = package_path + "/kitti_data/images/" + image_name + ".png"
    previous_data_path = package_path + "/kitti_data/images/" + prev_image_name + ".png"
    results_image_path = package_path + "/inferred_data/flow/" + image_name + ".flo.png"
    results_flo_path = package_path + "/inferred_data/flow/" + image_name + ".flo"
    print("Flo path: {}".format(flow_data_path))
    print("Current image: {}".format(current_data_path))
    print("Previous image: {}".format(previous_data_path))
    print("Inferred Data path: {}".format(results_image_path))

    optical_flow_gt = cv2.readOpticalFlow(flow_data_path)
    print(optical_flow_gt.shape)

    current_image = cv2.imread(current_data_path, cv2.IMREAD_UNCHANGED)
    previous_image = cv2.imread(previous_data_path, cv2.IMREAD_UNCHANGED)
    flow_net = FlowNetRos()

    flow_map = flow_net.analyse_flow(previous_image, current_image)
    rgb_flow = flow_net.flow2rgb(flow_map)

    cv2.imwrite(results_image_path, rgb_flow)
    cv2.writeOpticalFlow(results_flo_path, flow_map)

    print(flow_map)
    print(optical_flow_gt)

    print("Comparing results from flo files at\n{}\n{}".format(flow_data_path, results_flo_path))
    result = mse(optical_flow_gt, flow_map)
    print("MSE: {}". format(result))