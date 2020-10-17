"""
Mask R-CNN
Configurations and data loading code for MS COCO.

Copyright (c) 2017 Matterport, Inc.
Licensed under the MIT License (see LICENSE for details)
Written by Waleed Abdulla

------------------------------------------------------------

Usage: import the module (see Jupyter notebooks for examples), or run from
       the command line as such:

    # Train a new model starting from pre-trained COCO weights
    python3 coco.py train --dataset=/path/to/coco/ --model=coco

    # Train a new model starting from ImageNet weights. Also auto download COCO dataset
    python3 coco.py train --dataset=/path/to/coco/ --model=imagenet --download=True

    # Continue training a model that you had trained earlier
    python3 coco.py train --dataset=/path/to/coco/ --model=/path/to/weights.h5

    # Continue training the last model you trained
    python3 coco.py train --dataset=/path/to/coco/ --model=last

    # Run COCO evaluatoin on the last model you trained
    python3 coco.py evaluate --dataset=/path/to/coco/ --model=last
"""

import os
import sys
import time
import numpy as np
import imgaug  # https://github.com/aleju/imgaug (pip3 install imgaug)

# Download and install the Python COCO tools from https://github.com/waleedka/coco
# That's a fork from the original https://github.com/pdollar/coco with a bug
# fix for Python 3.
# I submitted a pull request https://github.com/cocodataset/cocoapi/pull/50
# If the PR is merged then use the original repo.
# Note: Edit PythonAPI/Makefile and replace "python" with "python3".
from pycocotools.coco import COCO
from pycocotools.cocoeval import COCOeval
from pycocotools import mask as maskUtils

import zipfile
import urllib.request
import shutil

# Root directory of the project
ROOT_DIR = os.path.abspath("../../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn.config import Config
from mrcnn import model as modellib, utils

import cv2
import copy
import random as rng
rng.seed(12345)

from mrcnn.visualize import display_instances

# Path to trained weights file
# COCO_MODEL_PATH = os.path.join(ROOT_DIR, "models/mask_rcnn_coco.h5")
package_path = "/home/jesse/Code/src/ros/src/multi_robot_perception/mask_rcnn/"
COCO_MODEL_PATH = "/home/jesse/Code/third_parties/Mask_RCNN/models/mask_rcnn_coco.h5"

# Directory to save logs and model checkpoints, if not provided
# through the command line argument --logs
DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")
DEFAULT_DATASET_YEAR = "2017"

############################################################
#  Configurations
############################################################
import tensorflow as tf 

gpus = tf.config.experimental.list_physical_devices('GPU')
print(gpus)
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

class InferenceConfig(CocoConfig):
    # Set batch size to 1 since we'll be running inference on
    # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1
    DETECTION_MIN_CONFIDENCE = 0


############################################################
#  Dataset
############################################################

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

    def __init__(self, weight_path =package_path+ "weights/mask_rcnn_coco.h5",
                        coco_annotations_path = package_path + "data/annotations", coco_year="2017"):
        self._weights_path = weight_path
        self._coco_annotations_path = coco_annotations_path
        self._coco_year = coco_year

        self._dataset = CocoDataset(self._coco_annotations_path, "val", self._coco_year)
        self._dataset.prepare()

        self.class_names = self._dataset.class_names

        self.config = InferenceConfig()
        self.config.display()

        #this logs default will have to change
        self.model = modellib.MaskRCNN(mode="inference", config=self.config,
                                    model_dir=DEFAULT_LOGS_DIR)

        # Select weights file to load
        #we could use imagenet? (see original code in mas_rcnn/coco/coco.py in main)
        print("Loading weights ", self._weights_path)
        self.model.load_weights(self._weights_path, by_name=True)
        print("Weights loaded")

    def analyse_image(self, image):
        print("Analysing Image")
        results = self.model.detect([image], verbose=1)

        # Visualize results
        r = results[0]
        return display_instances(image, r['rois'], r['masks'], r['class_ids'], 
                                    self.class_names, r['scores'])




if __name__ == '__main__':
    # Configurations

    # Load weights
    # print("Loading weights ", model_path)
    # model.load_weights(model_path, by_name=True)
    # dataset_val = CocoDataset()
    # coco = dataset_val.load_coco(args.dataset, val_type, year=args.year, return_coco=True, auto_download=args.download)
    # dataset_val.prepare()
    # print("Running COCO evaluation on {} images.".format(args.limit))
    #     evaluate_coco(model, dataset_val, coco, "bbox", limit=int(args.limit))
    # else:
    #     print("'{}' is not recognized. "
    #           "Use 'train' or 'evaluate'".format(args.command))
    rcnn = MaskRcnnRos()
    # image = cv2.imread("/home/jesse/Downloads/dash_cam_image.png", cv2.IMREAD_UNCHANGED)
    cap = cv2.VideoCapture(0)

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        print("Reading Frame")
        resized = cv2.resize(frame, (640,480), interpolation = cv2.INTER_AREA) 
        result = rcnn.analyse_image(resized)

        # Our operations on the frame come here
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        cv2.imshow('frame',result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

