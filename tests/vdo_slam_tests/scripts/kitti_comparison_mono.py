
from mono_depth_2.mono_depth_ros import MonoDepth2Ros
from vdo_slam_tests.utils import *
import rospy
import rospkg
import cv2
import sys

rospack = rospkg.RosPack()
package_path = rospack.get_path("vdo_slam_tests")


if __name__ == "__main__":
    rospy.init_node("kitti_comparison_mono")

    if len(sys.argv) > 1:
        image_name = sys.argv[1]
        if len(image_name) != 6:
            print("Provide image name without suffix. Eg '000001'")
            sys.exit(0)
    else:
        image_name = "000001"

    print("Using image: {}".format(image_name))

    depth_data_path = package_path + "/kitti_data/depth/" + image_name + ".png"
    image_data_path = package_path + "/kitti_data/images/" + image_name + ".png"
    results_data_path = package_path + "/inferred_data/depth/" + image_name + ".png"
    print("Depth path: {}".format(depth_data_path))
    print("Inferred Data path: {}".format(results_data_path))


    image1 = cv2.imread(image_data_path, cv2.IMREAD_UNCHANGED)
    #input is uint8

    mono_depth = MonoDepth2Ros()
    depth_image = mono_depth.analyse_depth(image1)
    print(depth_image)

    cv2.imwrite(results_data_path, depth_image)

    kitti_resut = cv2.imread(depth_data_path, cv2.IMREAD_UNCHANGED)

    print("Comparing results from depth files at\n{}\n{}".format(results_data_path, results_data_path))
    result = mse(kitti_resut, depth_image)
    print("MSE: {}", format(result))