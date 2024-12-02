import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.last_saved_time = time.time()

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= 2:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            filename = f'image_{int(current_time)}.jpg'
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f'Saved image: {filename}')
            self.last_saved_time = current_time

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
