from mask_rcnn.mask_rcnn_ros import MaskRcnnRos
from src.vdo_slam_tests.utils import *

import rospy
import rospkg
import cv2
import numpy as np
import sys


rospack = rospkg.RosPack()
package_path = rospack.get_path("vdo_slam_tests")


def convert_txtfile_numpy(file_path):
    return np.loadtxt(file_path)


if __name__ == "__main__":
    rospy.init_node("kitti_comparison_mask")

    if len(sys.argv) > 1:
        image_name = sys.argv[1]
        if len(image_name) != 6:
            print("Provide image name without suffix. Eg '000001'")
            sys.exit(0)
    else:
        image_name = "000001"

    print("Using image: {}".format(image_name))

    semantic_data_path = package_path + "/kitti_data/semantic/" + image_name + ".txt"
    image_data_path = package_path + "/kitti_data/images/" + image_name + ".png"
    results_data_path = package_path + "/inferred_data/semantic/" + image_name + ".txt"
    print("Semantic path: {}".format(semantic_data_path))
    print("Inferred Data path: {}".format(results_data_path))

    mask_rcnn = MaskRcnnRos()

    image1 = cv2.imread(image_data_path, cv2.IMREAD_UNCHANGED)
    print(image1.shape)

    pixel_mask, _, _ = mask_rcnn.analyse_image(image1)

    fil = open(results_data_path, "w")


    print(pixel_mask.shape)
    rows, cols = pixel_mask.shape
    for i in range(rows):
        for j in range(cols):
            value = str(pixel_mask[i][j])
            if j == (cols-1):
                fil.write(value)
            else:
                fil.write(value + " ")
        fil.write("\n")

    fil.close()

    inferred_resut = convert_txtfile_numpy(results_data_path)
    kitti_resut = convert_txtfile_numpy(semantic_data_path)
    print(kitti_resut.shape)

    print("Comparing results from txt files at\n{}\n{}".format(results_data_path, semantic_data_path))
    result = mse(kitti_resut, inferred_resut)
    print("MSE: {}", format(result))



