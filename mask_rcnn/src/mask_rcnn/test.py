import rospy
import ros_numpy
import rospkg
import sys

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from mask_rcnn.srv import MaskRcnnVdoSlam, MaskRcnnVdoSlamRequest
from mask_rcnn_ros import MaskRcnnRos
from threading import Thread
import cv2

rospack = rospkg.RosPack()

package_path = rospack.get_path("mask_rcnn")
sys.path.insert(0, package_path)

rospy.init_node("test_node")
analyse = rospy.ServiceProxy('maskrcnn/analyse_image', MaskRcnnVdoSlam)

class MaskRcnnThread(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.mask_rcnn = MaskRcnnRos()
        self.image = None
        self.is_updated = False

    def run(self):
        while not rospy.is_shutdown():
            if self.is_updated:
                image_copy = self.image.copy()
                mask = self.mask_rcnn.analyse_image(image_copy)
                self.is_updated = False
    
    def add_image(self, img):
        self.image = img
        self.is_updated = True

cam = cv2.VideoCapture(-1)
# mask_rcnn_thread = MaskRcnnThread()
# mask_rcnn_thread.start()
while not rospy.is_shutdown():
    ret_val, img = cam.read()
    # mask_rcnn_thread.add_image(img)
    
    image_msg = ros_numpy.msgify(Image, img, encoding='rgb8')
    image_msg.header = Header()
    image_msg.header.stamp = rospy.Time.now()

    srv = MaskRcnnVdoSlamRequest()
    srv.input_image = image_msg
    # print(request.input_image)

    response = analyse(srv)
    # rospy.sleep(1)
    # mask_img = response.output_viz
    # mask_img = ros_numpy.msgify(Image, mask_img, encoding='rgb8')
    # cv2.imshow("Mask", mask_img)
    # cv2.waitKey(1)
