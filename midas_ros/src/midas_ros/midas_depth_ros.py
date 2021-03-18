import cv2
import tensorflow as tf
import urllib.request
from rostk_pyutils.ros_cpp_communicator import RosCppCommunicator
from midas_ros.srv import MidasDepth, MidasDepthResponse
from sensor_msgs.msg import Image

import os
import sys
import cv2
import time
import rospkg
import rospy
import ros_numpy

import cv2


rospack = rospkg.RosPack()

package_path = rospack.get_path("midas_ros")
sys.path.insert(0, package_path)


class MidasRos(RosCppCommunicator):
    
    def __init__(self, model_path=package_path + "/src/midas_ros/models/", model_name="model_opt.tflite"):
        RosCppCommunicator.__init__(self)
        self.model_name = model_name
        self.model_folder = model_path
        self.model_path = model_path + self.model_name

        self.download_model()


        # load model
        self.interpreter = tf.lite.Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.input_shape = self.input_details[0]['shape']

        # constants for norm
        self._mean=[0.485, 0.456, 0.406]
        self._std=[0.229, 0.224, 0.225]

        self.midas_depth_service = rospy.Service("midasdepth/analyse_image",MidasDepth, self.midas_depth_service_callback)
        self.log_to_ros("midasdepth/analyse_image call ready")

    def midas_depth_service_callback(self, req):
        response = MidasDepthResponse()
        try:
            current_image = ros_numpy.numpify(req.current_image)
        except Exception as e:
            self.log_to_ros(str(e))
            response.success = False
            return response


        depth_image = self.analyse_image(current_image)

        output_image_msg = ros_numpy.msgify(Image, depth_image, encoding='mono16')

        del depth_image
        response.success = True
        response.output_image = output_image_msg

        return response

    def analyse_image(self, input_image):
        """[Estimates depth or RGB image using MiDaS model.]

        Args:
            input_image ([numpy array]): [Input image in BGR (OpenCV standard) form]

        Returns:
            [numpy array]: [Depth image of type CV16UC1]
        """
        # inference
        normalized_tensor = self._normalize_input(input_image)

        self.interpreter.set_tensor(self.input_details[0]['index'], normalized_tensor)
        self.interpreter.invoke()
        output = self.interpreter.get_tensor(self.output_details[0]['index'])
        output = output.reshape(256, 256)
                    
        # output file
        prediction = cv2.resize(output, (input_image.shape[1], input_image.shape[0]), interpolation=cv2.INTER_CUBIC)
        depth_min = prediction.min()
        depth_max = prediction.max()
        # uint8_output = (255 * (prediction - depth_min) / (depth_max - depth_min)).astype("uint8")

        #image normal is usual depth style imaage where closer objects are white
        image_normal =  (2**16 * (prediction - depth_min) / (depth_max - depth_min)).astype("uint16")

        inverted_image = cv2.bitwise_not(image_normal)
        return inverted_image

    def _normalize_input(self, input_image):
        """[Normalizes and preprocesses an input image, ready for inference]

        Args:
            input_image ([np.ndarray]): [Uint8 RGB image]

        Returns:
            [tf.tensor]: [Normalzied image as tf.tensor, type: tf.float32]
        """
        #input shoudl be unchanged uint8 RGB image
        imgf = input_image / 255.0
        imgf_resized = tf.image.resize(imgf, [256,256], method='bicubic', preserve_aspect_ratio=False)
        img_input = imgf_resized.numpy()

        img_input = (img_input - self._mean) / self._std
        reshape_img = img_input.reshape(1,256,256,3)
        return tf.convert_to_tensor(reshape_img, dtype=tf.float32)

    def download_model(self):
        if os.path.isfile(self.model_path):
            self.log_to_ros("Model file exists")
        else:
            self.log_to_ros("Downloading model...")
            url = "https://github.com/intel-isl/MiDaS/releases/download/v2_1/model_opt.tflite"
            urllib.request.urlretrieve(url, self.model_path)
            self.log_to_ros("Done")
        


def main():
    
    midas = MidasRos()

    cam = cv2.VideoCapture(0)
    while True:
        start_time = time.time()
        ret_val, img = cam.read()
        composite = midas.analyse_image(img)
        print("Time: {:.2f} s / img".format(time.time() - start_time))
        cv2.imshow("Depth detections", composite)
        if cv2.waitKey(1) == 27:
            break  # esc to quit
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
