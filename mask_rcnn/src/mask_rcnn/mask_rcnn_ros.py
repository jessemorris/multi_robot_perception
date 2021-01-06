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
import torch

from mask_rcnn.predictor import COCODemo

from mask_rcnn.srv import MaskRcnnVdoSlam, MaskRcnnVdoSlamResponse

from mask_rcnn.srv import MaskRcnnVisualise, MaskRcnnVisualiseResponse
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator

from sensor_msgs.msg import Image
import struct
import rospy
import random


import time

class MaskRcnnRos(RosCppCommunicator):

    def __init__(self, config_path = package_path + "src/mask_rcnn/configs/caffe2/e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml"):
        RosCppCommunicator.__init__(self)
        self.model_config_path = config_path
        cfg.merge_from_file(self.model_config_path)
        cfg.merge_from_list([])
        cfg.freeze()

        self.greyscale_palette = torch.tensor([2 ** 25 - 1])


        # prepare object that handles inference plus adds predictions on top of image
        self.coco_demo = COCODemo(
            cfg,
            confidence_threshold=0.7,
            show_mask_heatmaps=False,
            masks_per_dim=2,
            min_image_size=800
        )


        self.mask_rcnn_service = rospy.Service("mask_rcnn_service",MaskRcnnVdoSlam, self.mask_rcnn_service_callback)
        self.mask_rcnn_test_publisher = rospy.Publisher('mask_rcnn/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")


    def mask_rcnn_service_callback(self, req):
        response = MaskRcnnVdoSlamResponse()
        self.log_to_ros("Inside callback")
        try: 
            self.mask_rcnn_test_publisher.publish(req.input_image)

            input_image = ros_numpy.numpify(req.input_image)


            response_image, labels, label_indexs = self.analyse_image(input_image)
            self.log_to_ros(str(response_image.shape))

            output_image_msg = ros_numpy.msgify(Image, response_image, encoding='mono8')

            response.success = True
            # response.output_image = output_image_msg
            response.output_mask = output_image_msg
            response.labels = labels
            response.label_indexs = label_indexs
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
        """[Creates a mask using the original image and the set of predictions. The masks
        is a greyscale image where each instance object is non-zero and is used as the input
        for VDOSLAm]

        Args:
            image ([type]): [description]
            predictions ([type]): [description]

        Returns:
            [np.array, list, list]: [masked image, labels, label indexs]
        """        
        masks = predictions.get_field("mask").numpy()
        label_indexs = predictions.get_field("labels")
        label_indexs_numpy = label_indexs.numpy()
        labels = self.convert_label_index_to_string(label_indexs)

        

        number_of_masks = masks.shape[0]

        colors = self.generate_grayscale_values(label_indexs)

        width = image.shape[0]
        height = image.shape[1]
        blank_mask = np.zeros((width, height),np.uint8)

        for i in range(number_of_masks):
            # print(masks.shape)
            # pixel_mask = masks[i, 0, :, :].astype(np.uint8) * label_indexs_numpy[i]
            pixel_mask = masks[i, 0, :, :].astype(np.uint8) * colors[i]

            blank_mask += pixel_mask
        # self.log_to_ros(type(blank_mask))
        # self.log_to_ros(type(labels))
        # self.log_to_ros(type(label_indexs))
        

        return blank_mask, labels, label_indexs

    def convert_label_index_to_string(self, labels):
        return [self.coco_demo.CATEGORIES[i] for i in labels]

    def get_single_label_from_index(self, label):
        return self.coco_demo.CATEGORIES[label]

    def generate_grayscale_values(self, label_indexs):
        """[Generates n number of distinct values between 1 and 255 for each label. This should be 
        used for visualisation purposes only as VDOSLAM just needs a distinct value]

        Args:
            label_index ([List]): [List of label indexs generated from the predictor]

        Returns:
            [List]: [List of values]
        """  
        colors = label_indexs[:, None] * self.greyscale_palette
        colors = (colors % 255).numpy().astype("uint8")
        return colors





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
