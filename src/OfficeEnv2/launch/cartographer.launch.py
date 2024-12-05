from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

NUM_ROBOTS = 4

pkg_name = "OfficeEnv2"

def generate_launch_description():
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    nodes = [
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
    ]
    turtlebot3_cartographer_prefix = get_package_share_directory(pkg_name)
    for i in range(NUM_ROBOTS):
        cartographer_config_dir = LaunchConfiguration(pkg_name, default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
        configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default=f'cartographer_config_{i}.lua')

        nodes.append(DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        )

        nodes.append(DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'))

        nodes.append(Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name=f'cartographer_node_{i}',
            output='screen',
            namespace=f'robot_{i}',
            remappings=[
                # ('/tf', f'/robot_{i}/tf'),
                # ("/tf_static", f"/robot_{i}/tf_static"),
                ("/scan", f"/robot_{i}/scan"),
                ("/odom", f"/robot_{i}/odom"),
            ],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]
        ))

    # nodes.append(Node(
    #     package="OfficeEnv2",
    #     executable="swarm_to_one_bot",
    #     name=f"swarm_to_one_bot",
    #     output="screen",
    #     arguments=[str(NUM_ROBOTS)]
    # ))

        nodes.append(Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name=f'cartographer_occupancy_grid_node',
            output='screen',
            namespace=f'robot_{i}',
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec],
            remappings=[
                ("/submap_list", f"robot_{i}/submap_list"),
                ("/landmark_poses_list", f"robot_{i}/landmark_poses_list"),
                ("/trajectory_node_list", f"robot_{i}/trajectory_node_list"),
                ("/scan_matched_points2", f"robot_{i}/scan_matched_points2"),
                ("/constraint_list", f"robot_{i}/constraint_list"),
            ]
        ))

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        *nodes,
    ])
