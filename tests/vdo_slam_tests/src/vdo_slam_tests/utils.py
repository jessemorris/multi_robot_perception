import numpy as np

def mse(kitti_image, inferred_image):
    err = np.sum((kitti_image.astype("float") - inferred_image.astype("float")) ** 2)
    err /= float(kitti_image.shape[0] * kitti_image.shape[1])
    return err