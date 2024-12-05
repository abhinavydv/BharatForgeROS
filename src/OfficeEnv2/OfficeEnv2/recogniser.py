#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
import cv2, math
import time

MIN_DELTA_T_FOR_REFRESH = 0.8 # secs
CAMERA_FOV = 1.089 # rad

class Recogniser(Node):
    def __init__(self):
        super().__init__('recogniser')
        self.model = YOLO("yolov8m-seg.pt")

        # subscribe to rgb camera
        self.rgb_subscription = self.create_subscription(
            Image,
            '/robot_0/camera/image_raw',
            self.rgb_callback,
            10)
        
        # subscribe to depth camera
        self.depth_subscription = self.create_subscription(
            Image,
            '/robot_0/camera/depth/image_raw',  # Replace with your depth topic
            self.depth_callback,
            10
        )

        # Subscribe to the /odom topic
        self.subscription = self.create_subscription(
            Odometry,
            '/robot_0/odom',  # Replace with your odom topic name if different
            self.odom_callback,
            10
        )

        self.get_logger().info('Recogniser node started')

        self.last_saved_time_rgb = time.time()
        self.last_saved_time_depth = time.time()

        self.W = 0
        
        # current objs
        self.current_objects, self.current_objects_centers = [], []
        self.current_objects_dists = []
        self.current_mask, self.current_depth = [], []
        self.current_pixels, self.current_depth_map = [], []

        self.current_heading = 0
        self.current_position = []


    def rgb_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time_rgb >= MIN_DELTA_T_FOR_REFRESH:
            # get img_array
            img_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

            if self.W == 0:
                self.W = msg.width
            
            # recognise images
            current_results = self.model(img_array, verbose=False)
            
            # list of objs and their corresponding masks
            self.current_objects, self.current_objects_centers = [], []
            self.current_objects_dists = []
            self.current_mask, self.current_pixels = [], []

            # through each result,
            for result in current_results:
                # get name of current object
                try:
                    current_obj = result.names[int(result.boxes.cls.item())]
                    self.current_objects.append(current_obj)
                    #print(result.boxes, result.boxes[0])
                    #print('DATA:',result.masks.data, '\nLEN:', len(result.masks.data))

                    # append masks
                    for j, mask in enumerate(result.masks.data):
                        self.current_mask.append(cv2.resize(mask.numpy(), (msg.width, msg.height)))

                        # get "depth" pixels where objects exist
                        obj_pixels = np.argwhere(mask>0).numpy()

                        # get "rgb" pixels corresponding to depth pixels
                        self.current_pixels.append(obj_pixels)

                        # get object (depth cluster) center
                        self.current_objects_centers.append(np.mean(obj_pixels, axis=0))
                        
                except Exception as e:
                    pass

            self.last_saved_time_rgb = current_time

    
    def depth_callback(self, msg):
        current_time = time.time()

        if current_time - self.last_saved_time_depth >= MIN_DELTA_T_FOR_REFRESH:
            self.current_depth_map = []
            self.current_depth_image = []

            try:
                # get encoding type
                if msg.encoding == '16UC1':
                    dtype = np.uint16
                elif msg.encoding == '32FC1':
                    dtype = np.float32
                else:
                    self.get_logger().error(f'Unsupported depth encoding: {msg.encoding}')
                    return
                
                # Get depth image, assuming rgb info arrives before depth
                self.current_depth_image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
                
                # Get depth map
                self.current_depth_map = [
                    np.average(self.current_depth_image[self.current_pixels[i]]) for i in range(len(self.current_pixels))
                ]
                
                # process trigonometry and get object position w.r.t robot
                self.process()


            except Exception as e:
                self.get_logger().error(f'Failed to process depth image: {e}')
            
            self.last_saved_time_depth = current_time

    # get yaw (the heading) from the orientation quarternion
    def quaternion_to_yaw(self, orientation_q):
        """
        Convert a quaternion to a yaw angle (in radians).

        Args:
            orientation_q: geometry_msgs.msg.Quaternion

        Returns:
            yaw (float): The yaw angle in radians.
        """
        # Extract quaternion values
        x = orientation_q.x
        y = orientation_q.y
        z = orientation_q.z
        w = orientation_q.w

        # Compute yaw from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # Yaw (rotation around Z-axis)

        return yaw
    
    # updates position and orientation of robot
    def odom_callback(self, msg):
        # Extract the quaternion from the odometry message
        orientation_q = msg.pose.pose.orientation
        self.current_heading = self.quaternion_to_yaw(orientation_q)
        self.current_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]

    
    # process trigonometry to get an object's position with respect to the bot
    def process(self):
        # get heading
        heading = self.current_heading
        
        for i in range(len(self.current_objects)):
            # find the object's "offset" in the image
            w = self.current_objects_centers[i][0] - (self.W/2)
            W = self.W
            r = self.current_depth_map[i]
            # the delta theta
            delta_theta = math.atan2(w*math.tan(CAMERA_FOV/2), W/2)
            
            # the actual position of the object with respect to robot
            theta = heading + delta_theta
            
            # get corresponding x and y
            relative_xy = [r*math.sin(theta), r*math.cos(theta)]
            
            self.current_objects_dists.append(relative_xy)

            print('Object: ', self.current_objects[i], ' is @ ', relative_xy)



def main(args=None):
    rclpy.init(args=args)
    recogniser = Recogniser()
    rclpy.spin(recogniser)
    recogniser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()