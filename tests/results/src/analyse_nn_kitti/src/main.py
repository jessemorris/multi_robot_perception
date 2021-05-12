
from typing import List
import matplotlib.pyplot as plt
import numpy as np
import os
import random
from colour import Color
import argparse
import cv2



from mono_depth_2.mono_depth_ros import MonoDepth2Ros
from mask_rcnn.mask_rcnn_ros import MaskRcnnRos
from midas_ros.midas_depth_ros import MidasRos
from flow_net.flow_net_ros import FlowNetRos

_RESULTS_FOLDER = "/home/jesse/Code/src/ros/src/multi_robot_perception/tests/results/src/analyse_nn_kitti/figures/"
_DATA_FOLDER = "/home/jesse/Code/third_parties/VDO_SLAM_attempt_modifed/demo-kitti/"


K = np.array([[721.5377, 0, 721.5377],
    [0, 609.5593, 172.8540],
    [0, 0, 1]])

# D = np.array([-0.05400957120448697, -0.07842753582468161, 0.09596410068935728, -0.05152529532743679])
d =  np.array([0, 0, 0, 0])
image_size = (1242, 375)
R = np.eye(3,3)

def project_ponint(point, depth):
    pass




def get_coordinate_values(np_matrix, label_index):
    indices = np.where(np_matrix == label_index)
    stacked = np.stack(indices, axis=1)
    return stacked


def convert_txtfile_numpy(file_path):
    return np.loadtxt(file_path)


def IOU(image1, image2):
    intersection = np.logical_and(image1, image2)
    union = np.logical_or(image1, image2)
    iou_score = np.sum(intersection) / np.sum(union)
    return iou_score




def load_data(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename), cv2.IMREAD_UNCHANGED)
        if img is not None:
            images.append(img)
    return images

def load_semantics(folder):
    images = []
    for filename in os.listdir(folder):
        img = convert_txtfile_numpy(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images

def load_flow(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.readOpticalFlow(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images



_RGBD_FOLDER = _DATA_FOLDER + "image_0"
rgb_images = load_data(_RGBD_FOLDER)


def error_mono():
    print("Analysing Mono")
    data_folder = _DATA_FOLDER + "depth"
    monodepth = MonoDepth2Ros()

    print("Loading images from {}".format(data_folder))
    gt_depth_array = load_data(data_folder)

    errors = []
    
    for gt_depth, rgb in zip(gt_depth_array, rgb_images):
        inferred_depth = monodepth.analyse_depth(rgb)
        gt_depth[gt_depth == 0] = 1

        error_img = (gt_depth - inferred_depth)/gt_depth
        error = error_img.sum()/(error_img.shape[0] * error_img.shape[1])/100

        errors.append(error)

    avg = np.array(errors).sum()/len(errors)

    return errors, avg


        



def error_midas():
    print("Analysing Midas")
    data_folder = _DATA_FOLDER + "depth"
    midas = MidasRos()

    print("Loading images from {}".format(data_folder))
    gt_depth_array = load_data(data_folder)

    errors = []
    
    for gt_depth, rgb in zip(gt_depth_array, rgb_images):
        inferred_depth = midas.analyse_image(rgb)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        gt_depth[gt_depth == 0] = 1

        error_img = (gt_depth - inferred_depth)/gt_depth
        error = error_img.sum()/(error_img.shape[0] * error_img.shape[1])/100

        print("error was {}".format(error))

        errors.append(error)

    avg = np.array(errors).sum()/len(errors)
    return errors, avg



def error_mask():
    print("Analysing Mask")
    data_folder = _DATA_FOLDER + "semantic"
    mask_rcnn = MaskRcnnRos()

    print("Loading images from {}".format(data_folder))
    gt_array = load_semantics(data_folder)

    errors = []
    
    for gt_mask, rgb in zip(gt_array, rgb_images):
        response_image, semantic_objects = mask_rcnn.analyse_image(rgb)


        error = IOU(gt_mask, response_image)
        print("error was {}".format(error))

        errors.append(error)

    avg = np.array(errors).sum()/len(errors)
    return errors, avg
    


def error_flow():
    print("Analysing Flow")
    data_folder = _DATA_FOLDER + "flow"
    flow_net = FlowNetRos()

    print("Loading images from {}".format(data_folder))
    gt_array = load_flow(data_folder)

    errors = []
    previous_rbg = None
    
    for gt_flow, rgb in zip(gt_array, rgb_images):
        if previous_rbg is None:
            previous_rbg = rgb
            continue
        else:
            flow_mat = flow_net.analyse_flow(previous_rbg, rgb)



        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break


        gt_flow[gt_flow == 0] = 1

        error_img = ((gt_flow - flow_mat))/gt_flow
        error = error_img.sum()/(error_img.shape[0] * error_img.shape[1])/100
        error = abs(error)



        print("error was {}".format(error))

        errors.append(error)
        previous_rbg = rgb

    avg = np.array(errors).sum()/len(errors)
    return errors, avg



if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument('--nn')
    args = parser.parse_args()

    n_frames = len(rgb_images)
    x = np.linspace(0, n_frames-1, n_frames)

    # fig, ax = plt.subplots()



    if args.nn == "mono":
        errors, avg = error_mono()
        title = "Mono Depth 2"
    elif args.nn  == "midas":
        errors, avg = error_midas()
        title = "Midas"
    elif args.nn == "mask":
        errors, avg = error_mask()
        title = "Mask-RCNN"
    elif args.nn == "flow":
        errors, avg = error_flow()
        title = "Flow Net Lite"

    print(x)
    print(errors)
    #for flow
    if(len(x) != len(errors)):
        x = np.linspace(0, n_frames-2, n_frames-1)
    fig, ax = plt.subplots()
    ax.plot(x, errors)
    ax.axhline(y=avg, color='r', linestyle='-')
    ax.set_ylabel("% error")
    ax.set_xlabel("# Frames")
    ax.set_title(title)

    plt.show()

    

