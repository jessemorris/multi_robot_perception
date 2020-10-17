#!/usr/bin/env python
import cv2

import rospy

if __name__ == "__main__":
    rospy.init_node("rostk_plotting")
    print(rospy.get_param('/ros_toolkit/plotting/test'))
    rospy.spin()
    