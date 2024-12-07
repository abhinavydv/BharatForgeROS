import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
import sys


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        if len(sys.argv) < 2:
            print("Please provide the bot index")
            sys.exit()

        self.robot_index = int(sys.argv[1])

        self.odom_sub = Subscriber(self, Odometry, f'/robot_{self.robot_index}/odom')
        self.laser_sub = Subscriber(self, LaserScan, f'/robot_{self.robot_index}/scan')
        self.odom_laser_sync = ApproximateTimeSynchronizer([self.odom_sub, self.laser_sub], 10, 0.1)
        self.odom_laser_sync.registerCallback(self.odom_map_callback)

        print(f"Robot {self.robot_index} controller started")

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            f'/robot_{self.robot_index}/map',
            self.map_callback,
            10
        )

        self.task_subscription = self.create_subscription(
            Point,
            f'/robot_{self.robot_index}/task',
            self.task_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            f'/robot_{self.robot_index}/cmd_vel',
            10
        )

        self.complete_map = None
        self.task: Point = None

        self.planned_route = []
        self.next_pos_index = 0

        self.rl_obj = None

    def odom_map_callback(self, odom: Odometry, scan: LaserScan):
        """
        Control the bot here
        """
        if self.complete_map is None or self.task is None:
            return

    def move(self, vx, vy, w):
        """
        Move the bot
        """
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = w
        self.cmd_vel_publisher.publish(twist)

    def task_callback(self, msg: Point):
        """
        Update the task
        """
        self.task = msg
        self.plan_route()

    def plan_route(self):
        """
        Plan the route to the task
        """
        if self.complete_map is None or self.task is None:
            return

    def map_callback(self, msg: OccupancyGrid):
        """
        Update the map
        TODO: Implement map merging
        """
        print(msg.data)
        self.complete_map = msg.data


def main(args=None):
    rclpy.init(args=args)
    rc = RobotController()
    rclpy.spin(rc)
    rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
