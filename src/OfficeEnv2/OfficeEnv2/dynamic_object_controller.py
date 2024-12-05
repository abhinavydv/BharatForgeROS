import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DynamicObjectController(Node):
    def __init__(self):
        super().__init__('dynamic_object_controller')
        
        self.publisher = self.create_publisher(Twist, "/robot_0/cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.linear_speed = 0.1
        self.angular_speed = 0.1

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)
        # self.get_logger().info("Publishing: " + str(msg))

def main(args=None):
    rclpy.init(args=args)
    doc = DynamicObjectController()
    rclpy.spin(doc)
    doc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
