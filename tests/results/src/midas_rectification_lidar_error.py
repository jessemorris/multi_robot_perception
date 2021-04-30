import message_filters
from sensor_msgs.msg import Image, CameraInfo
from lidar_camera_projection.msg import ImagePixelDepth
from sensor_msgs.point_cloud2 import PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import ros_numpy
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R

import cv2

class DataCapture:

    def __init__(self):
        self.rgb_image_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
        self.unrectified_disp_sub = message_filters.Subscriber('/camera/depth/unrectified_disp', Image)
        self.rectified_disp_sub = message_filters.Subscriber('/camera/depth/image_raw', Image)
        self.lidar_points = message_filters.Subscriber('/lidar_camera_projections', ImagePixelDepth)

        self.lidar_pub = rospy.Publisher("/my_point_cloud", PointCloud2, queue_size=100)

        self.ts = message_filters.TimeSynchronizer([self.rgb_image_sub, 
                                                    self.unrectified_disp_sub,
                                                    self.rectified_disp_sub,
                                                    self.lidar_points], 1000)

        self.scaling_factor = 65536.0/70.0

        self.ts.registerCallback(self.callback)


    def callback(self, rgb_image_msg, unrectified_disp_msg, rectified_disp_msg, lidar_points_msg):
        rgb_image = ros_numpy.numpify(rgb_image_msg)
        unrectified_disp = ros_numpy.numpify(unrectified_disp_msg)
        rectified_disp = ros_numpy.numpify(rectified_disp_msg)

        # cloud = pcl.PointCloud_PointXYZ()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "gmsl_centre_link"
        # header.frame_id = "base_link"

        points = np.zeros((len(lidar_points_msg.data), 3))
        
        for i, lidar_pixel in enumerate(lidar_points_msg.data):
            # points[i][0] = lidar_pixel.z
            # points[i][1] = lidar_pixel.x
            # points[i][2] = lidar_pixel.y
            points[i][0] = lidar_pixel.x
            points[i][1] = lidar_pixel.y
            points[i][2] = lidar_pixel.z


            unrectified_depth_value = unrectified_disp[lidar_pixel.pixel_y][lidar_pixel.pixel_x]/self.scaling_factor

            rectified_depth_value = rectified_disp[lidar_pixel.pixel_y][lidar_pixel.pixel_x]/self.scaling_factor

            lidar_depth = lidar_pixel.z

            rospy.loginfo("Unrectified {} rectified {} gt {}".format(unrectified_depth_value, rectified_depth_value, lidar_depth))

            cv2.circle(rgb_image,(lidar_pixel.pixel_x, lidar_pixel.pixel_y), 3, (255,255,255), -1)

        # points = np.dot(points, rotation_matrix.T)

        cloud = pcl2.create_cloud_xyz32(header, points)
        self.lidar_pub.publish(cloud)


        # cloud.from_array(points)
        cv2.imshow("RGB", rgb_image)
        cv2.waitKey(1)





if __name__ == "__main__":
    rospy.init_node("midas_rectification_lidar_error")
    data_capture = DataCapture()

    rospy.spin()