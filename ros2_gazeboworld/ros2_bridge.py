import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import sys
import os
import cv2 as cv
from importlib import import_module
import transforms3d
import numpy as np
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
implementacao_opencv = import_module('implementacao_opencv')
apf = import_module('apf')

class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.current_position = None
        self.current_orientation = None

        # Subscrições e publicações
        self.camera_subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/model/meu_carrin/odometry', self.odom_callback, 50)
        self.lidar_subscription = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.position_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        implementacao_opencv.main(cv_image)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        # Verificar o quaternion recebido diretamente
        self.get_logger().info(f"Quaternion recebido: x={orientation_q.x}, y={orientation_q.y}, z={orientation_q.z}, w={orientation_q.w}")

        # Convertendo o quaternion para ângulo de Euler (yaw)
        (roll, pitch, yaw) = self.quaternion_to_euler(orientation_q)

        # Verificando o valor de yaw após conversão
        self.get_logger().info(f"Yaw após conversão: {yaw}")

        self.current_position = position
        self.current_orientation = yaw  # Armazenando o yaw (ângulo) para uso posterior

    def lidar_callback(self, msg):
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("erro")
            return

        lidar_data = msg.ranges
        goal = [7, 0]

        # Pegando os parâmetros necessários do msg
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        self.get_logger().info(f"Lidar data: {lidar_data[:5]}...")

        # Passando os parâmetros para o algoritmo
        forces = apf.algorithm(self.current_position, goal, lidar_data, angle_min, angle_increment, self.current_orientation)

        self.get_logger().info(f"Forças calculadas: {forces}")

        cmd_vel = Twist()
        cmd_vel.linear.x = forces[0]
        cmd_vel.angular.z = forces[1]

        self.position_publisher.publish(cmd_vel)

    def quaternion_to_euler(self, orientation_q):
        """Converte quaternion para Euler utilizando transforms3d"""
        # Criando um quaternion com a orientação
        quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]

        # Usando transforms3d para converter para Euler
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)

        # Retorna os ângulos de Euler
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    image_processing = ImageProcessing()
    rclpy.spin(image_processing)
    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
