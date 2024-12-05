from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

NUM_ROBOTS = 1

pkg_name = "OfficeEnv2"

def generate_launch_description():

    nodes = []

    for i in range(NUM_ROBOTS):
        slam_toolbox_config = {
            "use_sim_time": True,
            "slam_mode": True,
            "map_file_name": "",
            "resolution": 0.05,
            "publish_map": True,
            "map_update_interval": 2.0,
            "use_scan_matching": True,
            "use_scan_barycenter": False,
            "transform_tolerance": 0.1,
            "scan_topic": f"/robot_{i}/scan",
            "odom_topic": f"/robot_{i}/odom",
            "max_laser_range": 30.0,
            "minimum_time_interval": 0.0,
            "mode": "mapping",
            "queue_size": 100,
        }

        nodes.append(Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name=f"slam_toolbox_robot",
            namespace=f"robot_{i}",
            output="screen",
            parameters=[slam_toolbox_config],
        ))

    # nodes.append(Node(
    #     package="slam_toolbox",
    #     executable="sync_slam_toolbox_node",
    #     name="central_slam_toolbox",
    #     output="screen",
    #     parameters=[
    #         {
    #             "use_sim_time": True,
    #             "slam_mode": False,
    #             "map_merge_mode": "merge",
    #         },
    #     ],
    # ))

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        *nodes,
    ])
