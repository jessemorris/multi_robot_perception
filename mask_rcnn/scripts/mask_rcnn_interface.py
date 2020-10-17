import os
import sys
#TODO: relative paths
sys.path.append("/home/jesse/Code/src/ros/src/multi_robot_perception/mask_rcnn")
from src.mask_rcnn.mask_rcnn_ros import MaskRcnnRos
import rospy

if __name__ == '__main__':
    rospy.init_node('mask_rcnn_interface')
    mask_rcnn_ros = MaskRcnnRos()
    rospy.spin()
