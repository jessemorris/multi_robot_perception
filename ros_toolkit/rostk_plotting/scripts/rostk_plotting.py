#!/usr/bin/env python
import cv2
from pynput.keyboard import Listener, Key
import rospy



def press_callback(key):
    print(key.char)

if __name__ == "__main__":
    rospy.init_node("rostk_plotting")
    print(rospy.get_param('/ros_toolkit/plotting/test'))
    keyboard_listener = Listener(on_press=press_callback)
    keyboard_listener.start()


    

    rospy.spin()
    