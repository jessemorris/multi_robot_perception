from mono_depth_2.mono_depth_ros import MonoDepth2Ros
from mask_rcnn.mask_rcnn_ros import MaskRcnnRos
from vdo_slam_tests.utils import *
from sensor_msgs.msg import Image
import ros_numpy
import rospy
import rospkg
import cv2
import sys
import numpy as np

import time

rospack = rospkg.RosPack()
package_path = rospack.get_path("vdo_slam_tests")

def get_coordinate_values(np_matrix, label_index):
    indices = np.where(np_matrix == label_index)
    stacked = np.stack(indices, axis=1)

    return stacked


global np_image
np_image = None


def image_callback(data):
    global np_image
    np_image = ros_numpy.numpify(data)
    data_ready = True

def main():

    
    monodepth = MonoDepth2Ros()
    mask_rcnn = MaskRcnnRos()

    rospy.init_node('depth_tests')
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback, queue_size=10)
    while not rospy.is_shutdown():
        
        if np_image is None:
            continue
        start_time = time.time()
        img = np_image


        composite = monodepth.analyse_depth(img)
        response_image, labels, label_indexs, bounding_boxes = mask_rcnn.analyse_image(img)
        composite_scaled = composite
        display_image = mask_rcnn._generate_coloured_mask(response_image, labels, label_indexs)
        for i in range(len(labels)):
            index = label_indexs[i]
            label = labels[i]
            rect = bounding_boxes[i]
            coordinates = get_coordinate_values(response_image, index)
            avg_depth = 0
            x,y,w,h = rect
            for coord in coordinates:
                row = coord[0]
                col = coord[1]
                avg_depth += composite_scaled[row][col]

            avg_depth/= len(coordinates)
            avg_depth = 100 - avg_depth
            cv2.putText(img, 'Depth {} Class {}'.format(int(avg_depth), label), (x, y), cv2.FONT_HERSHEY_SIMPLEX,  
                   0.5, (0, 0, 255), 1, cv2.LINE_AA, False) 
            cv2.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)

        #same for background
        coordinates = get_coordinate_values(response_image, 0)
        avg_depth = 0
        for coord in coordinates:
            row = coord[0]
            col = coord[1]
            avg_depth += composite_scaled[row][col]
        print(len(coordinates))
        avg_depth/= len(coordinates)
        avg_depth = 100 - avg_depth
        print("AVG background depth is {}".format(avg_depth))





        print("Time: {:.2f} s / img".format(time.time() - start_time))
        cv2.imshow("COCO detections", img)
        cv2.imshow("Depth", composite)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()


