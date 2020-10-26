#!/usr/bin/env python
import cv2
import rospy
import sys

# from rostk_plotting.keypress_observer import KeypressObserver



if __name__ == "__main__":
    rospy.init_node("rostk_plotting")
    print(rospy.get_param('/ros_toolkit/plotting/snapshot_topics'))
    # keyboard_listener = Listener(on_press=press_callback)
    # keyboard_listener.start()
    print(sys.version)
    # keypress_observer = KeypressObserver()
    rospy.spin()
    