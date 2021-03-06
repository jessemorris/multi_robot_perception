### Lidar Projection and MonoDepth Comparison

To experimentally find the scaling factor for MonoDepth2 for this camera (ACFR-ITS dataset) we need to use ground truth pixel depth data. 

We subscribe to the projected points from the lidar and analyse the raw image that the points will be projected onto using Monodepth 2. Becuase the lidar is sparse we only take the points from the dense depthmap that have associated depth points from the lidar projection.

## Params (see launch file)

- no_images: Number of images to capture before ending the node
- time_delay: the amount of time to delay (in seconds) between taking images. Eg if no_images = 10 and time_delay = 3, 10 sets of depth analysis will be taken across 30 seconds (1 analysis every 3 seconds in realtime).

### Subscribed Topics
- subscribe to `/gmsl/*/camera_image/lidar_projected` to get a list of pixel coordinates and associated 3D points in space. NOTE: this topic is only in my version of the ITS dataset code. Additonal code was added to the `lidar_projection_node` in the `callback_lidar_camera` function that takes the _planepointsC_ and the lidar coordiantes and packages this into a custom package and advertises it out).
- subscribe to `/gmsl/*/camera_image` which is the raw image data. This is then analysed by MonoDepth to and the pixels at the corresponding locations saved.

This data is saved into a yaml file in `$(lidar_monodepth)/results/depth_mod.yaml` in the form

```yaml
image_id: 0 - no_images
    time: in seconds
    data: 
        [   lidar_x: float
            lidar_y: float
            lidar_z: float
            pixel_x: int
            pixel_y: int
            lidar_depth: float
            mono_depth: uint16,

            lidar_x: float
            lidar_y: float ...
        ]
        
```

The output of the monodepth analysis is saved as a uint16_t value as the ROS encoding used is `sensor_encodings::MONO16`.

### To Run

```
$ roslaunch python_service_starter python_service_starter.launch
$ roslaunch lidar_monodepth2 lidar_monodepth2.launch
$ roslaunch multi_camera_lidar multi_camera_lidar.launch bag_file_name:=<bag_file_name>
```


## Python Script for resulting scaling optimization