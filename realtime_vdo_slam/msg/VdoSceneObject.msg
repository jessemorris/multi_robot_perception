time time

geometry_msgs/Pose pose
geometry_msgs/Twist twist
#center (x,y) will be starting x, y which is the bottom left corner of the image
#size_x and size_y will be with and height of image. Follows the same generation as
# mask_rcnn::SemanticObject which this will replicate
vision_msgs/BoundingBox2D bounding_box
uint32 semantic_label
uint32 tracking_id
string label
uint32 uid

#use as placeholder for a cv::KeyPoint. We will only fill out x,y (leaving z = -1)
#point.x = u on the image plane and point.y = v on the image plane
geometry_msgs/Point[] keypoints