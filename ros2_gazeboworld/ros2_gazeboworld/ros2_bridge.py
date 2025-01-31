import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import sys
import os
import cv2 as cv
from importlib import import_module

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
implementacao_opencv = import_module('implementacao_opencv')
apf = import_module('apf')


class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.current_position = None

        self.camera_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)   
        self.lidar_subscription = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.position_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        implementacao_opencv.main(cv_image)
    
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = position
    
    def lidar_callback(self, msg):
        lidar_data = msg.ranges
        goal = [0, 10]
        forces = apf.algorithm(self.current_position, goal, lidar_data)
        cmd_vel = Twist()
        cmd_vel.linear.x = forces[0]
        cmd_vel.angular.y = forces[1]
        self.get_logger().info("publishing")
        self.position_publisher.publish(cmd_vel)



def main(args=None):
    rclpy.init(args=args)
    image_processing = ImageProcessing()
    rclpy.spin(image_processing)
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()