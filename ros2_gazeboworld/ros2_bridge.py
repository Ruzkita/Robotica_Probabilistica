import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseArray
from cv_bridge import CvBridge
import sys
import os
import cv2 as cv
from importlib import import_module
import transforms3d
import numpy as np

# Importa seus módulos
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
implementacao_opencv = import_module('implementacao_opencv')
apf = import_module('apf')


class ImageProcessing(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.current_position = None
        self.current_orientation = None

        # Subscrições e publicações
        self.camera_subscription = self.create_subscription(
            Image, '/camera', self.image_callback, 10)
        
        self.pose_subscriber = self.create_subscription(
            PoseArray, '/world/meu_mundo/dynamic_pose/info', self.pose_callback, 50)
        
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/lidar', self.lidar_callback, 10)
        
        self.position_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

    # Callback da câmera
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        implementacao_opencv.main(cv_image)

    # Nova callback para pegar a pose absoluta
    def pose_callback(self, msg: PoseArray):
        # Procura a pose do modelo "meu_carrin"
        # Aqui assumimos que o primeiro PoseArray corresponde ao carrinho
        if len(msg.poses) == 0:
            return
        
        pose = msg.poses[0]  # Ajuste se houver mais modelos e você precisar filtrar pelo nome
        self.current_position = pose.position
        orientation_q = pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation_q)
        self.current_orientation = yaw
        #self.get_logger().info(f"Pose: {yaw}")

    # Callback do LiDAR
    def lidar_callback(self, msg):
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("Posição ou orientação ainda não disponíveis")
            return

        lidar_data = msg.ranges
        goal = [7, 0]

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Passando os parâmetros para o algoritmo APF
        forces = apf.algorithm(self.current_position, goal, lidar_data, angle_min, angle_increment, self.current_orientation)

        cmd_vel = Twist()
        cmd_vel.linear.x = forces[0]
        cmd_vel.angular.z = forces[1]

        self.position_publisher.publish(cmd_vel)

    # Conversão de quaternion para Euler (roll, pitch, yaw)
    def quaternion_to_euler(self, orientation_q):
        quaternion = [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z]
        roll, pitch, yaw = transforms3d.euler.quat2euler(quaternion)
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    image_processing = ImageProcessing()
    rclpy.spin(image_processing)
    cv.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
