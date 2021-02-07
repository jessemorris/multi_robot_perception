Creates bagfile with necessary pre-processed and recorded data for plotting for VDO-SLAM paper results.


The Usyd Campus Dataset is being used to provide the raw sensor data for this project (http://its.acfr.usyd.edu.au/datasets/usyd-campus-dataset/)
From the dataset we need camera stream and odom. From here we process this with _mask_rcnn_, _flow_net_, and _mono_depth2_ which will act as the input for the VDO-SLAM algorithm.


- The relevant info from the USYD campus dataset will be captured (video and odom)
- Flow, Mask and Monodepth images will be sychronized with these timestamps and added to video streams
- output saved as a single bagfile.

The final bagfile will be in the form:

types:       nav_msgs/Odometry      [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/CameraInfo [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image      [060021388200f6f0f447d0fcd9c64743]
             tf2_msgs/TFMessage     [94810edda583a504dfda3829e70d7eec]
topics:      /camera/camera_info         1864 msgs    : sensor_msgs/CameraInfo
             /camera/depth/image_raw      841 msgs    : sensor_msgs/Image     
             /camera/flow/colour_map      841 msgs    : sensor_msgs/Image     
             /camera/flow/image_raw       841 msgs    : sensor_msgs/Image     
             /camera/mask/colour_mask     841 msgs    : sensor_msgs/Image     
             /camera/mask/image_raw       841 msgs    : sensor_msgs/Image     
             /camera/rgb/image_raw        841 msgs    : sensor_msgs/Image     
             /map                        6214 msgs    : nav_msgs/Odometry     
             /odom                       6214 msgs    : nav_msgs/Odometry     
             /tf                        41287 msgs    : tf2_msgs/TFMessage    
             /tf_static                     1 msg     : tf2_msgs/TFMessage
             
A current bag_file can be found at: https://drive.google.com/drive/folders/1qOocuHTlipVPB4-kU1hPvRlmEGeXkUja?usp=sharing

_/map_, _/odom_, _/tf_, _/tf_static_, _/camera/camera_info_ and _/camera/rgb/image_raw_ are all captured from the dataset. The others are pre-processed from here. _*/imag_raw_ topics are used directly as input to VDO-SLAM. The others are images for visulisation for each parent namespace (eg flow and mask).
