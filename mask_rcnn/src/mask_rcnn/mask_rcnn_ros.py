import cv2

import sys
from memory_profiler import profile
import rospkg


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
from mask_rcnn.srv import MaskRcnnLabelList, MaskRcnnLabelListResponse
from mask_rcnn.srv import MaskRcnnVisualise, MaskRcnnVisualiseResponse
from mask_rcnn.srv import MaskRcnnLabel, MaskRcnnLabelResponse
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator

from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D

import struct
import rospy
import random


import time


rospack = rospkg.RosPack()

package_path = rospack.get_path("mask_rcnn")
sys.path.insert(0, package_path)

#e2e_mask_rcnn_R_101_FPN_1x_caffe2
#e2e_mask_rcnn_X_101_32x8d_FPN_1x_caffe2
#e2e_mask_rcnn_X_101_32x8d_FPN_1x_caffe2
#e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml
class MaskRcnnRos(RosCppCommunicator):

    def __init__(self, config_path = package_path + "/src/mask_rcnn/configs/caffe2/e2e_mask_rcnn_R_50_FPN_1x_caffe2.yaml"):
        RosCppCommunicator.__init__(self)
        self.model_config_path = config_path
        cfg.merge_from_file(self.model_config_path)
        cfg.merge_from_list([])
        cfg.freeze()

        self._greyscale_palette = (2 * 25 - 1)
        self._colour_palette = np.array([2 ** 29 - 1, 2 ** 7 - 1, 2 ** 11 - 5])


        # prepare object that handles inference plus adds predictions on top of image
        self.coco_demo = COCODemo(
            cfg,
            confidence_threshold=0.75,
            show_mask_heatmaps=False,
            masks_per_dim=20,
            min_image_size=800
        )

        self._greyscale_colours = self._generate_grayscale_values()
        self._colours = self._generate_coloured_values()



        self.mask_rcnn_service = rospy.Service("maskrcnn/analyse_image",MaskRcnnVdoSlam, self.mask_rcnn_service_callback)
        self.mask_rcnn_label_service = rospy.Service("maskrcnn/request_label", MaskRcnnLabel, self.label_request_callback)
        self.mask_rcnn_label_list_service = rospy.Service("maskrcnn/request_label_list", MaskRcnnLabelList, self.label_list_request_callback)
        self.log_to_ros("Service call ready")


    @torch.no_grad()
    def mask_rcnn_service_callback(self, req):
        response = MaskRcnnVdoSlamResponse()
        try: 
            input_image = ros_numpy.numpify(req.input_image)

            response_image, labels, label_indexs, bounding_box_msgs = self.analyse_image(input_image)
            display_image = self._generate_coloured_mask(response_image, labels, label_indexs)
            # test_image = self.display_predictions(input_image)

            output_image_msg = ros_numpy.msgify(Image, response_image, encoding='mono8')
            display_image_msg = ros_numpy.msgify(Image, display_image, encoding='rgb8')

            response.success = True
            # response.output_image = output_image_msg
            response.output_mask = output_image_msg
            response.labels = labels
            response.label_indexs = label_indexs
            response.output_viz = display_image_msg
            response.bounding_boxes = bounding_box_msgs

            self.log_to_ros(response.labels)

            del response_image
            del labels
            del label_indexs
            return response



        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response

    def label_request_callback(self, req):
        response = MaskRcnnLabelResponse()
        labels = self.convert_label_index_to_string(req.label_index)
        response.labels = labels
        return response

    def label_list_request_callback(self, req):
        response = MaskRcnnLabelListResponse()
        response.labels = self.coco_demo.CATEGORIES
        return response

    @torch.no_grad()
    def display_predictions(self, image):
        #note: due to changes in predictions.py this will no longer work
        return self.coco_demo.run_on_opencv_image(image)

    @torch.no_grad()
    def analyse_image(self, image):
        predictions = self.coco_demo.compute_prediction(image)
        top_predictions = self.coco_demo.select_top_predictions(predictions)
        composite, labels, label_indexs =  self.create_pixel_masks(image, top_predictions)

        if top_predictions:
            #in form [x,y,w,h]
            bounding_boxes = []
            top_predictions = top_predictions.convert("xywh")
            boxes = top_predictions.bbox
            for box in boxes:
                xmin, ymin, w, h = box.split(1, dim=-1)

                #here we make bounding box msg types
                #we make pose x, y which is bottom left corner of image
                # leave theta as 0
                pose2d = Pose2D(int(xmin.numpy()[0]), int(ymin.numpy()[0]), 0)

                bounding_box_msg = BoundingBox2D()

                #size x and size y will be bounding box width and height respectively
                bounding_box_msg.size_x =  int(w.numpy()[0])
                bounding_box_msg.size_y = int(h.numpy()[0])

                bounding_box_msg.center = pose2d

                # rect = [int(xmin.numpy()[0]), int(ymin.numpy()[0]), int(w.numpy()[0]), int(h.numpy()[0]) ]
                bounding_boxes.append(bounding_box_msg)
            
            if len(bounding_boxes) < 1:
                bounding_boxes = [[]]

            return composite, labels, label_indexs, bounding_boxes
        else:
            return composite, labels, label_indexs, []


    def create_pixel_masks(self, image, predictions):
        """
        Adds the instances contours for each predicted object.
        Each label has a different color.

        Arguments:
            image (np.ndarray): an image as returned by OpenCV
            predictions (BoxList): the result of the computation by the model.
                It should contain the field `mask` and `labels`.
        """
        width = image.shape[0]
        height = image.shape[1]
        blank_mask = np.zeros((width, height),np.uint8)

        if predictions is None:
            return blank_mask, [], []
        masks = predictions.get_field("mask")
        label_indexs = predictions.get_field("labels").numpy()
        labels = self.convert_label_index_to_string(label_indexs)


        # colours = self.get_greyscale_colours(label_indexs)
        
        if masks.ndim < 3:
            masks = np.expand_dims(masks, axis=0)
            masks = np.expand_dims(masks, axis=0)

       #TODO: make sure there is a boarder around each mask so that they are definetely considered
        #separate objects
        # for mask, semantic_index in zip(masks, label_indexs):
        #     thresh = mask[0, :, :].astype(np.uint8) * semantic_index
        #     blank_mask += thresh

        #track semantic labels in this map
        # we want unique instance-level semantic labelling per class (so car: [1,2], person: [1,2])
        instance_track = {}

        for label_index in label_indexs:
            # there is at least one of these objects in the list
            instance_track[label_index] = 1

        for mask, semantic_index in zip(masks, label_indexs):
            label = instance_track[semantic_index]
            thresh = mask[0, :, :].astype(np.uint8) * label
            print(label)
            blank_mask += thresh
            instance_track[semantic_index] += 1



        composite = blank_mask

        return composite, labels, label_indexs

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
        return colors

    def _generate_coloured_values(self):
        numer_of_cats = len(self.coco_demo.CATEGORIES)  
        categories_index = np.linspace(0, numer_of_cats, numer_of_cats + 1)
        colors = [(np.multiply(index,self._colour_palette) % 255).astype('uint8') for index in categories_index]
        # colors = np.array(categories_index) % 255) * self._colour_palette
        # colors = (colors % 255).astype("uint8")
        return colors

    def generated_bounding_boxes(self, rgb_image, bounding_box_msgs):
        rgb_image_bb = rgb_image.copy()
        for bounding_box in bounding_box_msgs:
            cv2.rectangle(rgb_image_bb, (bounding_box.center.x, bounding_box.center.y),
                        (bounding_box.center.x + bounding_box.size_x, bounding_box.center.y + bounding_box.size_y), (0, 255, 255), 2)
        return rgb_image_bb

    def _generate_coloured_mask(self, mask, labels, labels_index):
        # mask =  np.expand_dims(mask, 2) 
        # mask = np.repeat(mask, 3, axis=2) # give the mask the same shape as your image
        coloured_img = np.zeros((mask.shape[0], mask.shape[1], 3))
        
        for index in labels_index:
            colour = self._colours[index]
            coloured_img[mask == index] = colour
            # mask.flatten()
            # np.where(mask == index, np.multiply(mask, colour), mask )
            # colored_mask = mask.reshape(coloured_img.shape)
            # print(colored_mask.shape)
            # coloured_img = coloured_img + colored_mask
        coloured_img = coloured_img.astype('uint8')
        return coloured_img





def main():
    
    maskrcnn = MaskRcnnRos()

    cam = cv2.VideoCapture(0)
    while True:
        start_time = time.time()
        ret_val, img = cam.read()
        # response_image, labels, label_indexs = maskrcnn.analyse_image(img)
        response_image, labels, label_indexs, bb = maskrcnn.analyse_image(img)
        display_image = maskrcnn._generate_coloured_mask(response_image, labels, label_indexs)
        bb_imagg = maskrcnn.generated_bounding_boxes(img, bb)

        # test_image = maskrcnn.display_predictions(img)
        print("Time: {:.2f} s / img".format(time.time() - start_time))
        cv2.imshow("COCO detections", bb_imagg)
        print(labels)
        # cv2.imshow("Preds", test_image)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
