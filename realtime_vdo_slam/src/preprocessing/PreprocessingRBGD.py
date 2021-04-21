"""[This is just the RGBD Image preprocessing pipeline recreatated in Python
due to the memory leak that seems to be plaging the servcice call. ]
"""

import rospy
import ros_numpy
from flow_net.flow_net_ros import FlowNetRos
from mask_rcnn.mask_rcnn_ros import MaskRcnnRos
import cv2
import rosbag

import message_filters
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, PointCloud2
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from realtime_vdo_slam.msg import VdoInput
from threading import RLock

class InputHolder:

    def __init__(self):
        self.time = None
        self.rgb_image = None
        self.rgb_msg = None

        self.depth_msg = None
        self.previous_image = None


class ProcessingRBGDPy:

    def __init__(self):
        self._flow_net = FlowNetRos()
        self._mask_rcnn = MaskRcnnRos()

        # self._rgb_topic = rospy.get_param("/vdo_preprocessing/rgb_topic")
        # self._depth_topic = rospy.get_param("/vdo_preprocessing/seg_topic")
        self._rgb_topic = "/camera/rgb/image_raw"
        self._depth_topic = "/camera/depth/image_raw"
        self.bag_write = rosbag.Bag("/media/jesse/Segate/Jesse/thesis/datasets/Week21_2018_processed/a0_mid_2_pc_all.bag", "w")
        self.lock = RLock()
        #note: we do not include the camera info becuase we assume that the images
        #have already been rectified from the usyd_camera_dataset_bag gen node
    
        self._rgb_sub_sync = message_filters.Subscriber(self._rgb_topic, Image)
        self._depth_sub_sync = message_filters.Subscriber(self._depth_topic, Image)
        self._ts = message_filters.TimeSynchronizer([self._rgb_sub_sync, self._depth_sub_sync], 1000)

        self.gps_sub = rospy.Subscriber("/gps", NavSatFix, self._gps_callback)
        self.map_sub = rospy.Subscriber("/map", Odometry, self._map_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self._odom_callback)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self._tf_callback)
        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self._tf_static_callback)
        self.pc_sub = rospy.Subscriber("/velodyne/points", PointCloud2, self._pc_callback)
        self.cam_info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self._cam_info_callback)

        # self._rgb_repub = rospy.Publisher("/vdoslam/input/camera/rgb/image_raw", Image, queue_size=10)

        # self._maskrcnn_raw = rospy.Publisher("/vdoslam/input/camera/mask/image_raw", Image, queue_size=10)
        # self._masrcnn_viz = rospy.Publisher("/vdoslam/input/camera/mask/colour_mask", Image, queue_size=10)

        # self._flownet_raw = rospy.Publisher("/vdoslam/input/camera/flow/image_raw", Image, queue_size=10)
        # self._flownet_viz = rospy.Publisher("/vdoslam/input/camera/flow/colour_map", Image, queue_size=10)

        # self._depth_repub = rospy.Publisher("/vdoslam/input/camera/depth/image_raw", Image, queue_size=10)

        # self._vdo_input_pub = rospy.Publisher("/vdoslam/input/all", VdoInput, queue_size=10)

        self._rgb_repub = "/vdoslam/input/camera/rgb/image_raw"

        self._maskrcnn_raw = "/vdoslam/input/camera/mask/image_raw"
        self._masrcnn_viz = "/vdoslam/input/camera/mask/colour_mask"

        self._flownet_raw = "/vdoslam/input/camera/flow/image_raw"
        self._flownet_viz = "/vdoslam/input/camera/flow/colour_map"

        self._depth_repub = "/vdoslam/input/camera/depth/image_raw"

        self._vdo_input_pub = "/vdoslam/input/all"

        self._previous_time = None
        self._current_time = None
        self._is_first = True

        self._previous_image = None
        self.video_time = None

        self._max_image_msgs = 312 #this is the number of image msgs in the bag file. Once we get to this number we quit the playback
        #and analyyse the messages and publish
        self._images_recieved = 0
        self.input_holder_array = []

        self._ts.registerCallback(self.dept_rgb_callback)

    def run(self):
        self._ts.registerCallback(self.dept_rgb_callback)
        while self._images_recieved <= self._max_image_msgs and not rospy.is_shutdown():
            pass

        self._rgb_sub_sync.sub.unregister()
        self._depth_sub_sync.sub.unregister()

        print("Publishing {} messages in input holder array".format(len(self.input_holder_array)))
        for i, input_holder in enumerate(self.input_holder_array):
            if rospy.is_shutdown():
                return
            rgb_image = input_holder.rgb_image
            rgb_msg = input_holder.rgb_msg
            depth_msg = input_holder.depth_msg
            previous_image = input_holder.previous_image

            vdo_input_msg = VdoInput()
            vdo_input_msg.header.stamp = input_holder.time
            rospy.loginfo("Input time: {}".format(input_holder.time))

            #run mask rcnn
            mask_rcnn_image, semantic_objects = self._mask_rcnn.analyse_image(rgb_image)
            mask_rcnn_display_image = self._mask_rcnn.generate_coloured_mask(mask_rcnn_image)

            mask_rcnn_msg = ros_numpy.msgify(Image, mask_rcnn_image, encoding="mono8")
            mask_rcnn_display_msg = ros_numpy.msgify(Image, mask_rcnn_display_image, encoding="rgb8")

            mask_rcnn_msg.header.stamp = input_holder.time
            mask_rcnn_display_msg.header.stamp = input_holder.time

            vdo_input_msg.mask = mask_rcnn_msg
            vdo_input_msg.semantic_objects = semantic_objects

            # self._masrcnn_viz.publish(mask_rcnn_display_msg)
            self.lock.acquire()
            self.bag_write.write(self._masrcnn_viz, mask_rcnn_display_msg)

            #run flownet
            flow_net_image = self._flow_net.analyse_flow(previous_image, rgb_image)
            flow_net_display_image = self._flow_net.flow2rgb(flow_net_image)

            flow_net_msg = ros_numpy.msgify(Image, flow_net_image, encoding='32FC2')
            flow_net_display_msg = ros_numpy.msgify(Image, flow_net_display_image, encoding='rgb8')

            flow_net_msg.header.stamp = input_holder.time
            flow_net_display_msg.header.stamp = input_holder.time

            vdo_input_msg.flow = flow_net_msg
            # self._flownet_viz.publish(flow_net_display_msg)
            self.bag_write.write(self._flownet_viz, flow_net_display_msg)

            #add and display original msgs
            rgb_msg.header.stamp = input_holder.time
            # self._rgb_repub.publish(rgb_msg)
            self.bag_write.write(self._rgb_repub, rgb_msg)
            vdo_input_msg.rgb = rgb_msg

            depth_msg.header.stamp = input_holder.time
            # self._depth_repub.publish(depth_msg)
            self.bag_write.write(self._depth_repub, depth_msg)
            vdo_input_msg.depth = depth_msg

            # self._vdo_input_pub.publish(vdo_input_msg)
            self.bag_write.write(self._vdo_input_pub, vdo_input_msg)
            self.lock.release()
            print("Published {} msg".format(i))

        self.bag_write.close()
        print("Done!")

    def _gps_callback(self, msg):
        self.lock.acquire()
        self.bag_write.write("/gps", msg)
        self.lock.release()

    def _map_callback(self, msg):
        self.lock.acquire()
        self.bag_write.write("/map", msg)
        self.lock.release()

    def _odom_callback(self, msg):
        self.lock.acquire()
        self.bag_write.write("/odom", msg)
        self.lock.release()

    def _tf_callback(self, msg):
        self.lock.acquire()
        self.bag_write.write("/tf", msg)
        self.lock.release()

    def _tf_static_callback(self, msg):
        self.lock.acquire()
        self.bag_write.write("/tf_static", msg)
        self.lock.release()

    def _pc_callback(self, msg):
        self.lock.acquire()
        rospy.loginfo(msg.header.stamp)
        self.bag_write.write("/velodyne/points", msg)
        self.lock.release()

    def _cam_info_callback(self, msg):
        self.lock.acquire()
        self.bag_write.write("/camera/camera_info", msg)
        self.lock.release()



    def dept_rgb_callback(self, rgb_msg, depth_msg):
        rgb_image = ros_numpy.numpify(rgb_msg)
        # depth_image = ros_numpy.numpify(depth_msg)

        if self._is_first:
            self._previous_image = rgb_image
            self._previous_time = rgb_msg.header.stamp
            self._is_first = False
            self.video_time = rospy.get_rostime()
            return
        else:
            # dt = rgb_msg.header.stamp - self._previous_time
            # self.video_time += dt
            rospy.loginfo( rospy.get_rostime())
            rospy.loginfo(rgb_msg.header.stamp)
            input_holder = InputHolder()
            input_holder.time = rgb_msg.header.stamp
            input_holder.rgb_image = rgb_image
            input_holder.previous_image = self._previous_image
            input_holder.rgb_msg = rgb_msg
            input_holder.depth_msg = depth_msg

            self.input_holder_array.append(input_holder)

            self._previous_image = rgb_image
            self._previous_time = rgb_msg.header.stamp
            # cv2.imshow("Prev", self._previous_image)
            # cv2.waitKey(1)

        self._images_recieved += 1
        print("Msgs recieved: {}".format(self._images_recieved))




if __name__ == "__main__":
    rospy.init_node("image_preprocessing")
    # rospy.set_param("/use_sim_time", True)
    #use sim time must be set to true before running this node
    #so that rospy.get_rostime() is zero and we can take the difference betweene ach message
    #this must be set on the command line with $rosparam set use_sim_time true
    processer = ProcessingRBGDPy()
    processer.run()
