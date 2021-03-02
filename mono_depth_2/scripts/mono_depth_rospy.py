import os
import sys

#TODO: 
sys.path.append("/home/jesse/Code/src/ros/src/multi_robot_perception/mono_depth_2")
from src.mono_depth_2.mono_depth_ros import MonoDepth2Ros
import rospy



if __name__ == '__main__':
    rospy.init_node('mono_depth_rospy')
    mono_depth_ros = MonoDepth2Ros()
    rospy.spin()