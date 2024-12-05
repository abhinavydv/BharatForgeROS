import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import sys
# from tf_transformations import quaternion_from_euler


class TFPublisher(Node):
    def __init__(self, bot_index):
        super().__init__('tf_publisher')

        print('TF Publisher node initialized')
        
        self.robot = f"/robot_{bot_index}"

        self.subscriber = self.create_subscription(
            Odometry,
            f'{self.robot}/odom',
            self.odom_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg: Odometry):
        data = msg.pose.pose

        transform = TransformStamped()
        
        # transform.transform.translation.x = data.position.x
        # transform.transform.translation.y = data.position.y
        # transform.transform.translation.z = data.position.z
        
        # transform.transform.rotation.x = data.orientation.x
        # transform.transform.rotation.y = data.orientation.y
        # transform.transform.rotation.z = data.orientation.z
        # transform.transform.rotation.w = data.orientation.w
        
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = "world"
        
        transform.child_frame_id = f"{self.robot}/odom"

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Please provide the robot index")
        return
    
    robot_index = int(sys.argv[1])
    
    tf_pub = TFPublisher(robot_index)
    rclpy.spin(tf_pub)
    tf_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
