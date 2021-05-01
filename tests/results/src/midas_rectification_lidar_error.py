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
from sklearn.metrics import mean_squared_error
import signal
import matplotlib.pyplot as plt

import cv2

global should_continue_
should_continue_= True


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

        self.scaling_factor = 1000 * 0.09 / 0.001

        self._unrectified_data_rmse = []
        self._rectified_data_rmse = []
        self._points_per_frame = []
        # self._gt_data = []
        self._time_stamps = []
        self._number_frames = -1

        self.ts.registerCallback(self.callback)

    def stop(self):
        self.rgb_image_sub.sub.unregister()
        self.unrectified_disp_sub.sub.unregister()
        self.rectified_disp_sub.sub.unregister()
        self.lidar_points.sub.unregister()

    def run_analysis(self):
        fig, (ax1, ax2) = plt.subplots(2)
        x = np.linspace(0, self._number_frames, self._number_frames+1)
        ax1.plot(x, self._unrectified_data_rmse)
        ax1.set_title("Disparity")
        ax2.plot(x, self._rectified_data_rmse)
        ax2.set_title("Disparity after scale and shift")
        # plt.plot(x, self._gt_data, label="Rectified")
        ax2.set_xlabel("# Frames")
        ax1.set_ylabel("RMSE")
        ax2.set_ylabel("RMSE")

        plt.legend()
        plt.show()

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

        unrecified_data = []
        rectified_data = []
        gt_data = []
        
        if len(lidar_points_msg.data) <= 0:
            rospy.loginfo("No data messages")
            return


        for i, lidar_pixel in enumerate(lidar_points_msg.data):
            # points[i][0] = lidar_pixel.z
            # points[i][1] = lidar_pixel.x
            # points[i][2] = lidar_pixel.y
            points[i][0] = lidar_pixel.x
            points[i][1] = lidar_pixel.y
            points[i][2] = lidar_pixel.z

            unrectified_depth_value = self.scaling_factor * 1/unrectified_disp[lidar_pixel.pixel_y][lidar_pixel.pixel_x]
            unrecified_data.append(unrectified_depth_value)

            rectified_depth_value = self.scaling_factor * 1/rectified_disp[lidar_pixel.pixel_y][lidar_pixel.pixel_x]
            # rospy.loginfo(rectified_depth_value)
            rectified_data.append(rectified_depth_value)

            lidar_depth = lidar_pixel.z
            # rospy.loginfo(lidar_depth)
            gt_data.append(lidar_depth)
            rospy.loginfo("rectified {} gt {}".format(rectified_depth_value,lidar_depth ))

            # rospy.loginfo("Unrectified {} rectified {} gt {}".format(unrectified_depth_value, rectified_depth_value, lidar_depth))

            # cv2.circle(rgb_image,(lidar_pixel.pixel_x, lidar_pixel.pixel_y), 3, (255,255,255), -1)
            self._time_stamps.append(rgb_image_msg.header.stamp)


        #run RMSE for frame
        self._unrectified_data_rmse.append(mean_squared_error(gt_data,unrecified_data))

        rectified_rmse = mean_squared_error(gt_data,rectified_data)
        rospy.loginfo(rectified_rmse)
        self._rectified_data_rmse.append(rectified_rmse)
        self._points_per_frame = len(lidar_points_msg.data)
        self._number_frames += 1


        cloud = pcl2.create_cloud_xyz32(header, points)
        self.lidar_pub.publish(cloud)


        # # cloud.from_array(points)
        # cv2.imshow("RGB", rgb_image)
        # cv2.waitKey(1)

def signal_handler(signal, frame):
    print("Stopping callbacks")
    global should_continue_
    should_continue_= False







if __name__ == "__main__":
    rospy.init_node("midas_rectification_lidar_error",  disable_signals=True)

    data_capture = DataCapture()

    signal.signal(signal.SIGINT, signal_handler)

    while not rospy.is_shutdown():
        if should_continue_ == False:
            break

    data_capture.stop()
    data_capture.run_analysis()
