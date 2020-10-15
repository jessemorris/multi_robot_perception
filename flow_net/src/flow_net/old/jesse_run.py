import argparse
from path import Path

import torch
import torch.backends.cudnn as cudnn
import torch.nn.functional as F
import models
from tqdm import tqdm

import torchvision.transforms as transforms
import flow_transforms
from imageio import imread, imwrite
import numpy as np
from util import flow2rgb

model_names = sorted(name for name in models.__dict__
                     if name.islower() and not name.startswith("__"))


parser = argparse.ArgumentParser(description='PyTorch FlowNet inference on a folder of img pairs',
                                 formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('data', metavar='DIR',
                    help='path to images folder, image names must match \'[name]0.[ext]\' and \'[name]1.[ext]\'')
parser.add_argument('pretrained', metavar='PTH', help='path to pre-trained model')
parser.add_argument('--output', '-o', metavar='DIR', default=None,
                    help='path to output folder. If not set, will be created in data folder')
parser.add_argument('--output-value', '-v', choices=['raw', 'vis', 'both'], default='both',
                    help='which value to output, between raw input (as a npy file) and color vizualisation (as an image file).'
                    ' If not set, will output both')
parser.add_argument('--div-flow', default=20, type=float,
                    help='value by which flow will be divided. overwritten if stored in pretrained file')
parser.add_argument("--img-exts", metavar='EXT', default=['png', 'jpg', 'bmp', 'ppm'], nargs='*', type=str,
                    help="images extensions to glob")
parser.add_argument('--max_flow', default=None, type=float,
                    help='max flow value. Flow map color is saturated above this value. If not set, will use flow map\'s max value')
parser.add_argument('--upsampling', '-u', choices=['nearest', 'bilinear'], default=None, help='if not set, will output FlowNet raw input,'
                    'which is 4 times downsampled. If set, will output full resolution flow map, with selected upsampling')
parser.add_argument('--bidirectional', action='store_true', help='if set, will output invert flow (from 1 to 0) along with regular flow')

device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")


@torch.no_grad()
def main():
    
    # Data loading code
    input_transform = transforms.Compose([
        flow_transforms.ArrayToTensor(),
        transforms.Normalize(mean=[0,0,0], std=[255,255,255]),
        transforms.Normalize(mean=[0.411,0.432,0.45], std=[1,1,1])
    ])

    pretrained_model = "pretrained/flownetc_EPE1.766.tar"
    img1_file = "images/kitti1.png"
    img2_file = "images/kitti2.png"
    # create model
    network_data = torch.load(pretrained_model)
    print(network_data)
    print("=> using pre-trained model '{}'".format(network_data['arch']))
    model = models.__dict__[network_data['arch']](network_data).to(device)
    model.eval()

    cudnn.benchmark = True

    #default
    div_flow = 20
    max_flow = None

    if 'div_flow' in network_data.keys():
        print("flow is {}".format(network_data['div_flow']))
        div_flow = network_data['div_flow']

    print("Image 1 file {}".format(img1_file))
    print("Image 2 file {}".format(img2_file))


    img1 = input_transform(imread(img1_file))
    print(type(img1))
    print(img1.shape)
    img2 = input_transform(imread(img2_file))
    input_var = torch.cat([img1, img2]).unsqueeze(0)


    # if args.bidirectional:
    #     # feed inverted pair along with normal pair
    #     inverted_input_var = torch.cat([img2, img1]).unsqueeze(0)
    #     input_var = torch.cat([input_var, inverted_input_var])

    input_var = input_var.to(device)
    # compute output
    output = model(input_var)
    print(output.shape)
    # if args.upsampling is not None:
    #     output = F.interpolate(output, size=img1.size()[-2:], mode=args.upsampling, align_corners=False)
    for suffix, flow_output in zip(['flow', 'inv_flow'], output):
        filename = "images/output"
        rgb_flow = flow2rgb(div_flow * output, max_value=max_flow)
        to_save = (rgb_flow * 255).astype(np.uint8).transpose(1,2,0)
        imwrite(filename + '.png', to_save)


if __name__ == '__main__':
    main()
