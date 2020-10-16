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

    

############################################################
#  COCO Evaluation
############################################################

def build_coco_results(dataset, image_ids, rois, class_ids, scores, masks):
    """Arrange resutls to match COCO specs in http://cocodataset.org/#format
    """
    # If no results, return an empty list
    if rois is None:
        print("ROIS are none")
        return []
    results = []
    for image_id in image_ids:
        print("Number of rois: {}".format(rois.shape[0]))
        # Loop through detections
        for i in range(rois.shape[0]):
            class_id = class_ids[i]
            score = scores[i]
            bbox = np.around(rois[i], 1)
            mask = masks[:, :, i]

            result = {
                "image_id": image_id,
                "category_id": dataset.get_source_class_id(class_id, "coco"),
                "bbox": [bbox[1], bbox[0], bbox[3] - bbox[1], bbox[2] - bbox[0]],
                "score": score,
                "segmentation": maskUtils.encode(np.asfortranarray(mask))
            }
            print(result)
            results.append(result)
    return results


def evaluate_coco(model, dataset, coco, eval_type="bbox", limit=0, image_ids=None):
    """Runs official COCO evaluation.
    dataset: A Dataset object with valiadtion data
    eval_type: "bbox" or "segm" for bounding box or segmentation evaluation
    limit: if not 0, it's the number of images to use for evaluation
    """
    # Pick COCO images from the dataset
    image_ids = image_ids or dataset.image_ids

    # Limit to a subset
    if limit:
        image_ids = image_ids[:limit]

    # Get corresponding COCO image IDs.
    coco_image_ids = [dataset.image_info[id]["id"] for id in image_ids]
    class_names = dataset.class_names

    print("Evaluating image ids: {}".format(coco_image_ids))

    t_prediction = 0
    t_start = time.time()

    results = []
    raw_results = []
    image = None
    for i, image_id in enumerate(image_ids):
        # Load image
        image = dataset.load_image(image_id)
        cv_image = image

        # Run detection
        t = time.time()
        r = model.detect([image], verbose=0)[0]
        t_prediction += (time.time() - t)
        raw_results.append(r)

        # Convert results to COCO format
        # Cast masks to uint8 because COCO tools errors out on bool
        image_results = build_coco_results(dataset, coco_image_ids[i:i + 1],
                                           r["rois"], r["class_ids"],
                                           r["scores"],
                                           r["masks"].astype(np.uint8))


            # color = (rng.randint(0,256), rng.randint(0,256), rng.randint(0,256))
            # if result["score"] > 0.85:
            #     bounding_box = result["bbox"]
            #     cv2.rectangle(cv_image, (int(bounding_box[0]), int(bounding_box[1])), (int(bounding_box[2]), int(bounding_box[3])), color, 2)

        # file_name = package_path + "results/" + str(image_id) + ".jpg"
        # print("saving file: {}".format(file_name))
        # cv2.imwrite(file_name, cv_image)

        results.extend(image_results)
        # results.append(image_results)
    r = raw_results[0]
    print(r)
    display_instances(image, r['rois'], r['masks'], r['class_ids'], 
                            class_names, r['scores'])
    



    # Load results. This modifies results with additional attributes.
    # coco_results = coco.loadRes(results)

    # Evaluate
    # cocoEval = COCOeval(coco, coco_results, eval_type)
    # cocoEval.params.imgIds = coco_image_ids
    # cocoEval.evaluate()
    # cocoEval.accumulate()
    # cocoEval.summarize()

    print("Prediction time: {}. Average {}/image".format(
        t_prediction, t_prediction / len(image_ids)))
    print("Total time: ", time.time() - t_start)


############################################################
#  Training
############################################################


if __name__ == '__main__':
    # Configurations

    # Load weights
    print("Loading weights ", model_path)
    model.load_weights(model_path, by_name=True)
    dataset_val = CocoDataset()
    coco = dataset_val.load_coco(args.dataset, val_type, year=args.year, return_coco=True, auto_download=args.download)
    dataset_val.prepare()
    print("Running COCO evaluation on {} images.".format(args.limit))
        evaluate_coco(model, dataset_val, coco, "bbox", limit=int(args.limit))
    else:
        print("'{}' is not recognized. "
              "Use 'train' or 'evaluate'".format(args.command))
    rcnn = MaskRcnnRos()

