# Copyright Niantic 2019. Patent Pending. All rights reserved.
#
# This software is licensed under the terms of the Monodepth2 licence
# which allows for non-commercial use only, the full terms of which are made
# available in the LICENSE file.

from __future__ import absolute_import, division, print_function

import os
import sys
import glob
import argparse
import numpy as np
import PIL.Image as pilImage
# from PIL import Image
import matplotlib as mpl
import matplotlib.cm as cm

import torch
from torchvision import transforms, datasets

from src.mono_depth_2.networks import DepthDecoder
from src.mono_depth_2.networks import ResnetEncoder
from src.mono_depth_2.layers import disp_to_depth
from src.mono_depth_2.utils import download_model_if_doesnt_exist

from std_msgs.msg import String
from sensor_msgs.msg import Image

from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator
from mono_depth_2.srv import MonoDepth, MonoDepthResponse
import cv2
import time
import rospkg
import rospy
import ros_numpy


rospack = rospkg.RosPack()

package_path = rospack.get_path("mono_depth_2")

class MonoDepth2Ros(RosCppCommunicator):
    def __init__(self, model_path= package_path + "/src/mono_depth_2/models/", model_name = "mono_640x192"):
        RosCppCommunicator.__init__(self)
        self.model_name = model_name

        self.model_path = model_path + self.model_name

        if torch.cuda.is_available():
            self.device = torch.device("cuda")
        else:
            self.device = torch.device("cpu")

        download_model_if_doesnt_exist(self.model_name, self.model_path)
        self.log_to_ros("-> Loading model from {}".format(self.model_path))
        self.encoder_path = os.path.join(self.model_path, "encoder.pth")
        self.depth_decoder_path = os.path.join(self.model_path, "depth.pth")

        # LOADING PRETRAINED MODEL
        self.log_to_ros("Loading pretrained encoder")
        self.encoder = ResnetEncoder(18, False)
        self.loaded_dict_enc = torch.load(self.encoder_path, map_location=self.device)

        # extract the height and width of image that this model was trained with
        self.feed_height = self.loaded_dict_enc['height']
        self.feed_width = self.loaded_dict_enc['width']
        self.filtered_dict_enc = {k: v for k, v in self.loaded_dict_enc.items() if k in self.encoder.state_dict()}
        self.encoder.load_state_dict(self.filtered_dict_enc)
        self.encoder.to(self.device)
        self.encoder.eval()

        self.log_to_ros("Loading pretrained decoder")
        self.depth_decoder = DepthDecoder(
            num_ch_enc=self.encoder.num_ch_enc, scales=range(4))

        self.loaded_dict = torch.load(self.depth_decoder_path, map_location=self.device)
        self.depth_decoder.load_state_dict(self.loaded_dict)

        self.depth_decoder.to(self.device)
        self.depth_decoder.eval()

        #set up service calls
        self.mono_depth_service = rospy.Service("mono_depth_service",MonoDepth, self.mono_depth_service_callback)
        self.flow_net_test_publisher = rospy.Publisher('mono_depth/test', Image, queue_size=10)
        self.log_to_ros("Service call ready")

    @torch.no_grad()
    def mono_depth_service_callback(self, req):
        response = MonoDepthResponse()
        try:
            current_image = ros_numpy.numpify(req.current_image)
            self.log_to_ros((current_image.shape))
            # self.flow_net_test_publisher.publish(image)

            # previous_image = self.bridge.imgmsg_to_cv2(req.previous_image, "bgr8")
            # current_image = self.bridge.imgmsg_to_cv2(req.current_image, "bgr8")
        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response


        depth_image = self.analyse_depth(current_image)

        output_image_msg = ros_numpy.msgify(Image, depth_image, encoding='mono8')
        response.success = True
        response.output_image = output_image_msg

        return response


    def analyse_depth(self, input_image):
        """[Estimates depth of monocular image]

        Args:
            input_image ([numpy array]): [Input image in BGR (OpenCV standard) form]

        Returns:
            [numpy array]: [Depth image of type CV8UC1]
        """        
        image = pilImage.fromarray(input_image)
    
        original_width, original_height = image.size
        image = image.resize((self.feed_width, self.feed_height), pilImage.LANCZOS)
        self.log_to_ros(image)
        self.log_to_ros("Input image type {}".format(type(image)))
        image = transforms.ToTensor()(image).unsqueeze(0)
        self.log_to_ros(image.size())
        self.log_to_ros("Input image type {}".format(type(image)))
        #size is eventually torch.Size([1, 3, 192, 640]) we can give it cv image and then convert to torch
        self.log_to_ros(image.size())

        # PREDICTION
        image = image.to(self.device)
        features = self.encoder(image)
        outputs = self.depth_decoder(features)

        disp = outputs[("disp", 0)]
        disp_resized = torch.nn.functional.interpolate(
            disp, (original_height, original_width), mode="bilinear", align_corners=False)

        # Saving colormapped depth image
        depth_image = disp_resized.squeeze().cpu().numpy()
        return depth_image

    def depth_image_to_colourmap(self, depth_image):
        """[Converts the depth image to a colour mapping for visualiation]

        Args:
            depth_image ([np array]): [Depth image as output by self.analyse_depth]

        Returns:
            [np array]: [Colour image]
        """        
        vmax = np.percentile(depth_image, 95)
        normalizer = mpl.colors.Normalize(vmin=depth_image.min(), vmax=vmax)
        mapper = cm.ScalarMappable(norm=normalizer, cmap='magma')
        return (mapper.to_rgba(depth_image)[:, :, :3] * 255).astype(np.uint8)



# def test_simple(args):
#     """Function to predict for a single image or folder of images
#     """
#     # assert args.model_name is not None, \
#     #     "You must specify the --model_name parameter; see README.md for an example"

#     # if torch.cuda.is_available() and not args.no_cuda:
#     #     device = torch.device("cuda")
#     # else:
#     #     device = torch.device("cpu")

#     if torch.cuda.is_available():
#         device = torch.device("cuda")
#     else:
#         device = torch.device("cpu")

#     model_name = "mono_640x192"
#     download_model_if_doesnt_exist(model_name)
#     model_path = os.path.join("models", model_name)
#     print("-> Loading model from ", model_path)
#     encoder_path = os.path.join(model_path, "encoder.pth")
#     depth_decoder_path = os.path.join(model_path, "depth.pth")

    # # LOADING PRETRAINED MODEL
    # print("   Loading pretrained encoder")
    # encoder = networks.ResnetEncoder(18, False)
    # loaded_dict_enc = torch.load(encoder_path, map_location=device)

    # # extract the height and width of image that this model was trained with
    # feed_height = loaded_dict_enc['height']
    # feed_width = loaded_dict_enc['width']
    # filtered_dict_enc = {k: v for k, v in loaded_dict_enc.items() if k in encoder.state_dict()}
    # encoder.load_state_dict(filtered_dict_enc)
    # encoder.to(device)
    # encoder.eval()

    

    # FINDING INPUT IMAGES
    # if os.path.isfile(args.image_path):
    #     # Only testing on a single image
    #     paths = [args.image_path]
    #     output_directory = os.path.dirname(args.image_path)
    # elif os.path.isdir(args.image_path):
    #     # Searching folder for images
    #     paths = glob.glob(os.path.join(args.image_path, '*.{}'.format(args.ext)))
    #     output_directory = args.image_path
    # else:
    #     raise Exception("Can not find args.image_path: {}".format(args.image_path))

    # print("-> Predicting on {:d} test images".format(len(paths)))

    # cam = cv2.VideoCapture(0)

    # img = cv2.imread("/home/jesse/Code/src/ros/src/multi_robot_perception/mono_depth_2/src/test_image.jpg")
    # PREDICTING ON EACH IMAGE IN TURN
#     with torch.no_grad():
#         while True:
#         # while True:
#         #     start_time = time.time()
#         #     ret_val, img = cam.read()
#         #     composite = maskrcnn.analyse_image(img)
#         #     print("Time: {:.2f} s / img".format(time.time() - start_time))
#         #     cv2.imshow("COCO detections", composite)
#         #     if cv2.waitKey(1) == 27:
#         #         break  # esc to quit
#         # cv2.destroyAllWindows()

#             # for idx, image_path in enumerate(paths):

#             # if image_path.endswith("_disp.jpg"):
#                 # don't try to predict disparity for a disparity image!

            
#             # ret_val, img = cam.read()
#             img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

#             input_image = Image.fromarray(img)
#             # # img_numpy = np.ascontiguousarray(np.array(img)[:, :, ::-1].astype(np.float32) * (1.0 / 255.0))
#             # img_numpy = np.ascontiguousarray(np.array(img)[:, :, ::-1].astype(np.float32))

#             # img_numpy = cv2.resize(img_numpy, (feed_width,feed_height), interpolation = cv2.INTER_AREA)
#             # original_height, original_width, channels = img_numpy.shape



#             # print(img_numpy.shape)


#             # Load image and preprocess
#             # input_image = pil.open("assets/test_image.jpg").convert('RGB')
#             # print("Input image type {}".format(type(input_image)))
#             original_width, original_height = input_image.size
#             input_image = input_image.resize((feed_width, feed_height), pil.LANCZOS)
#             print(input_image)
#             print("Input image type {}".format(type(input_image)))
#             input_image = transforms.ToTensor()(input_image).unsqueeze(0)
#             print(input_image.size())
#             print("Input image type {}".format(type(input_image)))
#             #size is eventually torch.Size([1, 3, 192, 640]) we can give it cv image and then convert to torch
#             print(input_image.size())

#             # PREDICTION
#             start_time = time.time()
#             input_image = input_image.to(device)
#             features = encoder(input_image)
#             outputs = depth_decoder(features)

#             disp = outputs[("disp", 0)]
#             disp_resized = torch.nn.functional.interpolate(
#                 disp, (original_height, original_width), mode="bilinear", align_corners=False)

#             # Saving numpy file
#             # output_name = os.path.splitext(os.path.basename(image_path))[0]
#             # name_dest_npy = os.path.join(output_directory, "{}_disp.npy".format(output_name))
#             # scaled_disp, _ = disp_to_depth(disp, 0.1, 100)
#             # np.save(name_dest_npy, scaled_disp.cpu().numpy())

#             # Saving colormapped depth image
#             disp_resized_np = disp_resized.squeeze().cpu().numpy()
#             vmax = np.percentile(disp_resized_np, 95)
#             normalizer = mpl.colors.Normalize(vmin=disp_resized_np.min(), vmax=vmax)
#             mapper = cm.ScalarMappable(norm=normalizer, cmap='magma')
#             colormapped_im = (mapper.to_rgba(disp_resized_np)[:, :, :3] * 255).astype(np.uint8)
#             print("Time: {:.2f} s / img".format(time.time() - start_time))

#             print(type(colormapped_im))
#             cv2.imshow("Depth detections", colormapped_im)
#             if cv2.waitKey(1) == 27:
#                 break  # esc to quit
#             # im = pil.fromarray(colormapped_im)

#             # name_dest_im = os.path.join(output_directory, "{}_disp.jpeg".format(output_name))
#             # im.save(name_dest_im)

#             # print("   Processed {:d} of {:d} images - saved prediction to {}".format(
#             #     idx + 1, len(paths), name_dest_im))
#         cv2.destroyAllWindows()

#     print('-> Done!')


# if __name__ == '__main__':
#     # args = parse_args()
#     args = None
#     test_simple(args)
