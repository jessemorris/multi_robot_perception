import numpy as np
import sys

# def rmse(kitti_image, inferred_image):
#     return np.sqrt(np.mean((kitti_image-inferred_image)**2))

# def mse(kitti_image, inferred_image):
#     err = np.sum((kitti_image.astype("float") - inferred_image.astype("float")) ** 2)
#     err /= float(kitti_image.shape[0] * kitti_image.shape[1])
#     return err

def mse(kitti_image, inferred_image):
    err = np.sqrt(np.mean((kitti_image.astype("float") - inferred_image.astype("float")) ** 2))
    err /= float(kitti_image.shape[0] * kitti_image.shape[1])
    return err

def parse_image_name(argv):
    if len(argv) > 1:
        image_name = argv[1]
        if len(image_name) != 6:
            print("Provide image name without suffix. Eg '000001'")
            sys.exit(0)
    else:
        image_name = "000001"

    print("Using image: {}".format(image_name))
    return image_name