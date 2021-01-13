import cv2

import sys


package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/mask_rcnn/"
sys.path.insert(0, package_path)
from maskrcnn_benchmark.config import cfg
from maskrcnn_benchmark.utils import cv2_util
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

        self._greyscale_palette = (2 * 25 - 1)


        # prepare object that handles inference plus adds predictions on top of image
        self.coco_demo = COCODemo(
            cfg,
            confidence_threshold=0.7,
            show_mask_heatmaps=False,
            masks_per_dim=2,
            min_image_size=800
        )

        self._greyscale_colours = self._generate_grayscale_values()



        self.mask_rcnn_service = rospy.Service("mask_rcnn_service",MaskRcnnVdoSlam, self.mask_rcnn_service_callback)
        self.mask_rcnn_test_publisher = rospy.Publisher('mask_rcnn/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")


    def mask_rcnn_service_callback(self, req):
        response = MaskRcnnVdoSlamResponse()
        self.log_to_ros("Inside callback")
        try: 
            input_image = ros_numpy.numpify(req.input_image)

            response_image, labels, label_indexs = self.analyse_image(input_image)
            test_image = self.display_predictions(input_image)

            output_image_msg = ros_numpy.msgify(Image, response_image, encoding='mono8')
            test_image_msg = ros_numpy.msgify(Image, test_image, encoding='rgb8')
            self.mask_rcnn_test_publisher.publish(test_image_msg)

            response.success = True
            # response.output_image = output_image_msg
            response.output_mask = output_image_msg
            response.labels = labels
            response.label_indexs = label_indexs
            return response



        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response

    def display_predictions(self, image):
        return self.coco_demo.run_on_opencv_image(image)

    def analyse_image(self, image):
        predictions = self.coco_demo.compute_prediction(image)
        top_predictions = self.coco_demo.select_top_predictions(predictions)
        return self.create_pixel_masks(image, top_predictions)
       
        # result = image.copy()

    def create_pixel_masks(self, image, predictions):
        """
        Adds the instances contours for each predicted object.
        Each label has a different color.

        Arguments:
            image (np.ndarray): an image as returned by OpenCV
            predictions (BoxList): the result of the computation by the model.
                It should contain the field `mask` and `labels`.
        """
        masks = predictions.get_field("mask").numpy()
        label_indexs = predictions.get_field("labels").numpy()
        labels = self.convert_label_index_to_string(label_indexs)

        width = image.shape[0]
        height = image.shape[1]

        colours = self.get_greyscale_colours(label_indexs)
        
        blank_mask = np.zeros((width, height, 1),np.uint8)

        for mask, colour in zip(masks, colours):
            thresh = mask[0, :, :, None].astype(np.uint8) * colour
            
        
            blank_mask += thresh
            # contours, hierarchy = cv2_util.findContours(
            #     thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            # )
            # # contours = contours[0] if len(contours) == 2 else contours[1]
            # image = cv2.drawContours(image, contours, -1, color, 3)

        composite = blank_mask

        return composite, labels, label_indexs

    # def create_pixel_masks(self, image, predictions):
    #     """[Creates a mask using the original image and the set of predictions. The masks
    #     is a greyscale image where each instance object is non-zero and is used as the input
    #     for VDOSLAm]

    #     Args:
    #         image ([type]): [description]
    #         predictions ([type]): [description]

    #     Returns:
    #         [np.array, list, list]: [masked image, labels, label indexs]
    #     """        
    #     masks = predictions.get_field("mask").numpy()
    #     label_indexs = predictions.get_field("labels")
    #     label_indexs_numpy = label_indexs.numpy()
    #     labels = self.convert_label_index_to_string(label_indexs)
    #     self.log_to_ros(labels)

        

    #     number_of_masks = masks.shape[0]
    #     self.log_to_ros("Number of masks {}".format(number_of_masks))

    #     colors = self.generate_grayscale_values(label_indexs)

    #     width = image.shape[0]
    #     height = image.shape[1]
    #     blank_mask = np.zeros((width, height, 1),np.uint8)

    #     for i in range(number_of_masks):
    #         # print(masks.shape)
    #         # pixel_mask = masks[i, 0, :, :].astype(np.uint8) * label_indexs_numpy[i]
    #         print( masks[i, 0, :, :].astype(np.uint8))
    #         pixel_mask = masks[i, 0, :, :].astype(np.uint8) * colors[i]

    #         # blank_mask += pixel_mask
    #         blank_mask = cv2.add(blank_mask, pixel_mask)
    #     # self.log_to_ros(type(blank_mask))
    #     # self.log_to_ros(type(labels))
    #     # self.log_to_ros(type(label_indexs))
        

    #     return blank_mask, labels, label_indexs

    def convert_label_index_to_string(self, labels):
        return [self.coco_demo.CATEGORIES[i] for i in labels]

    def get_single_label_from_index(self, label):
        return self.coco_demo.CATEGORIES[label]

    def get_greyscale_colours(self, label_index):
        return self._greyscale_colours[label_index]

    def _generate_grayscale_values(self):
        """[Generates n number of distinct values between 1 and 255 for each label. This should be 
        used for visualisation purposes only as VDOSLAM just needs a distinct value]

        Returns:
            [List]: [List of values]
        """
        numer_of_cats = len(self.coco_demo.CATEGORIES)  
        categories_index = np.linspace(0, numer_of_cats, numer_of_cats + 1)
        colors = np.array(categories_index) * self._greyscale_palette
        colors = (colors % 255).astype("uint8")
        print(type(colors))
        return colors





def main():
    
    maskrcnn = MaskRcnnRos()

    cam = cv2.VideoCapture(0)
    while True:
        start_time = time.time()
        ret_val, img = cam.read()
        # response_image, labels, label_indexs = maskrcnn.analyse_image(img)
        response_image = maskrcnn.analyse_image(img)
        # test_image = maskrcnn.display_predictions(img)
        print("Time: {:.2f} s / img".format(time.time() - start_time))
        cv2.imshow("COCO detections", response_image)
        # cv2.imshow("Preds", test_image)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
