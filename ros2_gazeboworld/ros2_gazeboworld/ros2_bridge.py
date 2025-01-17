import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import sys
import os
import cv2 as cv
from importlib import import_module

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
implementacao_opencv = import_module('implementacao_opencv')
bidirecional_a_star = import_module('bidirecional_a_star')


class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            implementacao_opencv.main(cv_image)
        except Exception as e:
            self.get_logger().error(f"Erro: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_processing = ImageProcessing()
    rclpy.spin(image_processing)
    cv.destroyAllWindows()
    rclpy.shutdown

if __name__ == "__main__":
    main()