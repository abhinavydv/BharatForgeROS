from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

pkg_name = "OfficeEnv"

NUM_ROBOTS = 4

def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", "my_robot.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", "robot_rviz_config.rviz"]
    )

    robots = []
    spawns = []
    tf_s = []

    for i in range(NUM_ROBOTS):
        robots.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path'), f" robot_name:=robot_{i}"])
                }],
                remappings=[('/tf', f'/robot_{i}/tf'), ("/robot_description", f"/robot_{i}/robot_description")],
            ),
        )

        spawns.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', f'robot_{i}/robot_description', '-robot_namespace', f'/robot_{i}', '-entity', f'robot_{i}', '-x', str(-i), '-y', '1.0', '-z', '0.0']
            )
        )

        tf_s.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[str(-i), '0.0', '0.0', '0.0', '0.0', '0.0', '/world', f'/robot_{i}/odom']
            )
        )

    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),

        DeclareLaunchArgument(
            'urdf_path',
            default_value=urdf_path,
            description='Path to the URDF file'
        ),
        DeclareLaunchArgument(
            'rviz_config_path',
            default_value=rviz_config_path,
            description='Path to the RViz config file'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config_path')]
        ),
        *tf_s,
        *robots,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={'world': PathJoinSubstitution([FindPackageShare(pkg_name), 'worlds', 'updated_office.world'])}.items()
        ),
        *spawns,
    ])
