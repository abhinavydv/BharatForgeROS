import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
import os

class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        
        print('MapSaver node initialized')

    def listener_callback(self, og):
        current_time = time.time()
        
        # occupancy grid is 2D array. Convert to python list
        og_data = og.data
        og_data_list = []
        for i in range(0, len(og_data)):
            og_data_list.append(og_data[i])
        
        # print(og_data_list)
        
        arr = np.array(og_data_list)
        
        print(arr.shape, og.info.width, og.info.height)

        arr = arr.reshape((og.info.height, og.info.width))
        
        print(os.path.dirname(os.path.curdir))
        cv2.imwrite(f'map_images/map_{current_time}.png', arr)

def main(args=None):
    rclpy.init(args=args)
    image_saver = MapSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
