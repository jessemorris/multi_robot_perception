### Contains nodes to pre-prrocess image data stream and then run the vdo-algorithm on this output

## Structure
__image_preprocessing__ node

_input_ : single camera steam topic (defined in _realtime_vdo.yaml_)
_output_:

- "vdoslam/input/camera/rgb/image_raw" -> same as the image defined by the input topic (N x M x 3, RGB)
- "vdoslam/input/camera/mask/image_raw" -> image mask with pixels labelled 0...N where 0 notates the background. (N x M x 1, MONO8)
- "vdoslam/input/camera/mask/colour_mask" -> image mask but each mask will be coloured coded for easy visualisation (N x M x 3, RGB)
- "vdoslam/input/camera/flow/image_raw" -> flow map of the previus and current frames. Output form is the same as .flo files use by OpenCV (N x M x 2, Float32)
- "vdoslam/input/camera/flow/colour_map" -> flow map represted in 3D colour space for easy visulisation (N x M x 3, RGB)
- "vdoslam/input/camera/depth/image_raw" -> estimated depth map of input image (N x M x 1, MONO16)


__ros_vdo_slam__ node

_input_ : the above */image_raw topics
_output_: some visulisation (TODO)