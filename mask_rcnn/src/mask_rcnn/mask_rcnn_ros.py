

import os
import sys
import time
import numpy as np
import imgaug  # https://github.com/aleju/imgaug (pip3 install imgaug)

from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
from pycocotools import mask as maskUtils

import zipfile
import urllib.request
import shutil

ROOT_DIR = os.path.abspath("../../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn.config import Config
from mrcnn import model as modellib, utils

import cv2
import copy


from mrcnn.visualize import display_instances
from sensor_msgs.msg import Image
import ros_numpy
import rospy
import struct

from mask_rcnn.srv import MaskRcnn, MaskRcnnResponse



# Path to package
#TODO: relative
package_path ="/home/jesse/Code/src/ros/src/multi_robot_perception/mask_rcnn/"

# Directory to save logs and model checkpoints, if not provided
# through the command line argument --logs
#TODO: fix logs dont want to fill log file
DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")
print("Defauly logs dir {}".format(DEFAULT_LOGS_DIR))
DEFAULT_DATASET_YEAR = "2017"

############################################################
#  Configurations
############################################################
import tensorflow as tf 

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        # Currently, memory growth needs to be the same across GPUs
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        logical_gpus = tf.config.experimental.list_logical_devices('GPU')
        print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
    except RuntimeError as e:
        # Memory growth must be set before GPUs have been initialized
        print(e)





class CocoConfig(Config):
    """Configuration for training on MS COCO.
    Derives from the base Config class and overrides values specific
    to the COCO dataset.
    """
    # Give the configuration a recognizable name
    NAME = "coco"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 1

    # Uncomment to train on 8 GPUs (default is 1)
    # GPU_COUNT = 8

    # Number of classes (including background)
    NUM_CLASSES = 1 + 80  # COCO has 80 classes


#TODO: can we change these values to make them faster
class InferenceConfig(CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    DETECTION_MIN_CONFIDENCE = 0


class CocoDataset(utils.Dataset):

    def __init__(self, dataset_annotations_dir, subset, year):
        super().__init__()
        #TODO: update comments
        self.coco = COCO("{}/instances_{}{}.json".format(dataset_annotations_dir, subset, year))

        #load class ids
        class_ids = sorted(self.coco.getCatIds())
        for i in class_ids:
            self.add_class("coco", i, self.coco.loadCats(i)[0]["name"])


class MaskRcnnRos:

    def __init__(self, weight_path =package_path+ "src/mask_rcnn/weights/mask_rcnn_coco.h5",
                        coco_annotations_path = package_path + "datasets/annotations", coco_year="2017"):
        self._weights_path = weight_path
        self._coco_annotations_path = coco_annotations_path
        self._coco_year = coco_year


        self._dataset = CocoDataset(self._coco_annotations_path, "val", self._coco_year)
        self._dataset.prepare()

        self.class_names = self._dataset.class_names

        self.config = InferenceConfig()
        # self.config.display()

        try:
            self._write_fd = int(os.getenv("mask_rcnn_PY_WRITE_FD"))
            self.write_pipe = os.fdopen(self._write_fd, 'wb', 0)
        except:
            self.write_pipe = None


        if self.write_pipe == None:
            #do stuff
            #TODO if not constructed just print
            pass


        #this logs default will have to change
        self.model = modellib.MaskRCNN(mode="inference", config=self.config,
                                    model_dir=DEFAULT_LOGS_DIR)

        # Select weights file to load
        #we could use imagenet? (see original code in mas_rcnn/coco/coco.py in main)
        self.log_to_ros("Loading weights {}".format(self._weights_path))
        self.model.load_weights(self._weights_path, by_name=True)
        self.log_to_ros("Weights loaded")

        #set up service calls
        self.mask_rcnn_service = rospy.Service("mask_rcnn_service",MaskRcnn, self.mask_rcnn_service_callback)
        self.mask_rcnn_test_publisher = rospy.Publisher('mask_rcnn/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")

    def log_to_ros(self, msg):
        if self.write_pipe is None:
            print(msg)
        else:
            msg_size = struct.pack('<I', len(msg))
            self.write_pipe.write(msg_size)
            self.write_pipe.write(msg.encode("utf-8"))

    def mask_rcnn_service_callback(self, req):
        response = MaskRcnnResponse()
        try: 
            input_image = ros_numpy.numpify(req.input_image)
            response_image = self.analyse_image(input_image)

            output_image_msg = ros_numpy.msgify(Image, response_image, encoding='rgb8')
            self.mask_rcnn_test_publisher.publish(output_image_msg)

            response.success = True
            response.output_image = output_image_msg
            return response


        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response

    def analyse_image(self, image):
        # print("Analysing Image")
        results = self.model.detect([image], verbose=1)

        # Visualize results
        r = results[0]
        return display_instances(image, r['rois'], r['masks'], r['class_ids'], 
                                    self.class_names, r['scores'])




# if __name__ == '__main__':
    

#     rcnn = MaskRcnnRos()
#     # image = cv2.imread("/home/jesse/Downloads/dash_cam_image.png", cv2.IMREAD_UNCHANGED)
#     cap = cv2.VideoCapture(0)

#     while(True):
#         # Capture frame-by-frame
#         ret, frame = cap.read()
#         print("Reading Frame")
#         resized = cv2.resize(frame, (640,480), interpolation = cv2.INTER_AREA) 
#         result = rcnn.analyse_image(resized)

#         # Our operations on the frame come here
#         # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Display the resulting frame
#         cv2.imshow('frame',result)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break

# # When everything done, release the capture
#     cap.release()
#     cv2.destroyAllWindows()

