#!/usr/bin/env python
import cv2
import rospy
import roslib
import sys

# package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/ros_toolkit/rostk_plotting"
# sys.path.insert(0, package_path)
import rostk_plotting.keypress_listener


if __name__ == "__main__":
    rospy.init_node("rostk_plotting")
    print(rospy.get_param('/ros_toolkit/plotting/snapshot_topics'))
    # keyboard_listener = Listener(on_press=press_callback)
    # keyboard_listener.start()
    print(sys.version)
    # keypress_observer = KeypressObserver()
    rospy.spin()
    