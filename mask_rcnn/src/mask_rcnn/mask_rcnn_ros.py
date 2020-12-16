# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved.
import argparse
import cv2

import sys


package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/mask_rcnn/"
sys.path.insert(0, package_path)
from maskrcnn_benchmark.config import cfg
import ros_numpy
import os
import numpy as np
import math
import matplotlib.path

from mask_rcnn.predictor import COCODemo
from mask_rcnn.srv import MaskRcnn, MaskRcnnResponse
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator

from sensor_msgs.msg import Image
import struct
import rospy


import time

class MaskRcnnRos(RosCppCommunicator):

    def __init__(self, config_path = package_path + "src/mask_rcnn/configs/caffe2/e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml"):
        RosCppCommunicator.__init__(self)
        self.model_config_path = config_path
        cfg.merge_from_file(self.model_config_path)
        cfg.merge_from_list([])
        cfg.freeze()


        # prepare object that handles inference plus adds predictions on top of image
        self.coco_demo = COCODemo(
            cfg,
            confidence_threshold=0.7,
            show_mask_heatmaps=False,
            masks_per_dim=2,
            min_image_size=800
        )


        self.mask_rcnn_service = rospy.Service("mask_rcnn_service",MaskRcnn, self.mask_rcnn_service_callback)
        self.mask_rcnn_test_publisher = rospy.Publisher('mask_rcnn/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")


    def mask_rcnn_service_callback(self, req):
        response = MaskRcnnResponse()
        self.log_to_ros("Inside callback")
        try: 
            self.mask_rcnn_test_publisher.publish(req.input_image)

            input_image = ros_numpy.numpify(req.input_image)


            response_image = self.analyse_image(input_image)
            self.log_to_ros(str(response_image.shape))

            output_image_msg = ros_numpy.msgify(Image, response_image, encoding='rgb8')

            response.success = True
            # response.output_image = output_image_msg
            response.output_image = output_image_msg
            return response



        except Exception as e:
            print("here")
            self.log_to_ros(str(e))
            response.success = False
            return response

    # def analyse_image(self, image):
    #     return self.coco_demo.run_on_opencv_image(image)

    def analyse_image(self, image):
        predictions = self.coco_demo.compute_prediction(image)
        top_predictions = self.coco_demo.select_top_predictions(predictions)
        return self.create_pixel_masks(image, top_predictions)

        # result = image.copy()

    def create_pixel_masks(self, image, predictions):
        masks = predictions.get_field("mask").numpy()
        labels = predictions.get_field("labels")

        colors = self.coco_demo.compute_colors_for_labels(labels).tolist()

        for mask, color in zip(masks, colors):
            thresh = mask[0, :, :, None].astype(np.uint8)
            #contours are like my polygon
            contours, hierarchy = cv2.findContours(
                thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            self.log_to_ros(type(contours))
            self.log_to_ros(len(contours))
            self.log_to_ros(type(contours[0]))

            # left = np.min(polygon, axis=0)
            # right = np.max(polygon, axis=0)
            # x = np.arange(math.ceil(left[0]), math.floor(right[0])+1)
            # y = np.arange(math.ceil(left[1]), math.floor(right[1])+1)
            # xv, yv = np.meshgrid(x, y, indexing='xy')
            # points = np.hstack((xv.reshape((-1,1)), yv.reshape((-1,1))))

            # path = matplotlib.path.Path(polygon)
            # mask = path.contains_points(points)
            # mask.shape = xv.shape

            image = cv2.drawContours(image, contours, -1, color, 3)

        composite = image

        return composite






def main():
    
    maskrcnn = MaskRcnnRos()

    cam = cv2.VideoCapture(0)
    while True:
        start_time = time.time()
        ret_val, img = cam.read()
        composite = maskrcnn.analyse_image(img)
        print("Time: {:.2f} s / img".format(time.time() - start_time))
        cv2.imshow("COCO detections", composite)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
