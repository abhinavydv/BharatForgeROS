import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from cartographer_ros_msgs.msg import SubmapList
import sys

class SwarmToOneNode(Node):
    def __init__(self, num_robots):
        super().__init__('sarwm_to_one')

        print('SwarmToOne node initialized')

        self.subscribers = []

        # for i in range(num_robots):
        #     self.subscribers.append(self.create_subscription(
        #         SubmapList,
        #         f'/robot_{i}/submap_list',
        #         self.map_callback,
        #         10
        #     ))

        # self.pub = self.create_publisher(SubmapList, '/submap_list', 10)

        self.sub_map_list = []

        self.pubs = []
        for i in range(num_robots):
            self.subscribers.append(self.create_subscription(
                Odometry,
                f'/robot_{i}/odom',
                lambda msg: self.odom_callback(msg, i),
                10
            ))

            self.pubs.append(self.create_publisher(Odometry, f'/robot_{i}/filtered_odom', 10))

    def map_callback(self, msg: SubmapList):
        print('Received submap list')
        # sub = SubmapList()

        # sub.submap.extend(msg.submap)

        self.pub.publish(msg)
        
    def odom_callback(self, msg: Odometry, i):
        if -1e-3 < msg.pose.pose.position.x < 1e-3 and -1e-3 < msg.pose.pose.position.y < 1e-3 and -1e-3 < msg.pose.pose.position.x < 1e-3:
            return
        self.pubs[i].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Please provide the number of robots")
        return
    
    swarm_to_one = SwarmToOneNode(int(sys.argv[1]))
    rclpy.spin(swarm_to_one)
    swarm_to_one.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
