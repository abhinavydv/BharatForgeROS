import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class RobotController(Node):
    def __init__(self):
        super().__init__('debug')

        self.read_once = False

        self.subscription = self.create_subscription(
            PointCloud2,
            '/robot_0/camera/points',
            self.point_cloud_callback,
            10
        )

    def point_cloud_callback(self, msg: PointCloud2):
        # print the point cloud data
        if not self.read_once:
            self.read_once = True
            # for p in read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #     print(p)
            for m in range(0, len(msg.data), 32):
                print(msg.data[m+16], msg.data[m+17], msg.data[m+18])

def main(args=None):
    rclpy.init(args=args)
    rc = RobotController()
    rclpy.spin(rc)
    rc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
