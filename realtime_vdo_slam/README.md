### Contains nodes to pre-prrocess image data stream and then run the vdo-algorithm on this output

## Structure
__image_preprocessing__ node

_input_ : single camera steam topic (defined in _realtime_vdo.yaml_)

_output_:

- _/camera/rgb/image_raw_ -> same as the image defined by the input topic (N x M x 3, RGB)
- _/camera/mask/image_raw_ -> image mask with pixels labelled 0...N where 0 notates the background. (N x M x 1, MONO8)
- _/camera/mask/colour_mask_ -> image mask but each mask will be coloured coded for easy visualisation (N x M x 3, RGB)
- _/camera/flow/image_raw_ -> flow map of the previus and current frames. Output form is the same as .flo files use by OpenCV (N x M x 2, Float32)
- _/camera/flow/colour_map_ -> flow map represted in 3D colour space for easy visulisation (N x M x 3, RGB)
- _/camera/depth/image_raw_ -> estimated depth map of input image (N x M x 1, MONO16)


__ros_vdo_slam__ node

_input_ : the above */image_raw topics

_output_: some visulisation (TODO)

## Running ##

### From Raw data

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch realtime_vdo_slam vdo_preprocessing.launch 
$ roslaunch realtime_vdo_slam realtime_vdo_slam_launch.launch 
```

Change the input video stream _vdo_preprocessing_ in __realtime_vdo.yaml__.

### From Preprocessed Data
You can use the __vdo_bag_generation__ node to generate a bag file with all the necessary processed data and simply run this and the vdo slam node.

Currently, this node has been structured to generate data using the USYD Campus Dataset (http://its.acfr.usyd.edu.au/datasets/usyd-campus-dataset/) and so will not work with any input data.

See https://drive.google.com/drive/folders/1qOocuHTlipVPB4-kU1hPvRlmEGeXkUja?usp=sharing for an already preprocessed bagfile.

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch realtime_vdo_slam realtime_vdo_slam_launch.launch 
$ rosbag play <path_to_file>.bag
```


