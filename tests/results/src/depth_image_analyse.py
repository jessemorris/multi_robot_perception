import numpy as np
import cv2

root_folder = "/home/jesse/Code/src/ros/src/multi_robot_perception/VDO_SLAM/demo-kitti/"

depth_image_path = root_folder + "depth/000012.png"

depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)


print("Max value {}".format(depth_image.max()))
print("Min value {}".format(depth_image.min()))

depth_image_invert = cv2.bitwise_not(depth_image)

cv2.imshow("Depth Invert", depth_image_invert)
cv2.waitKey()