from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

pkg_name = "OfficeEnv"

def generate_launch_description():
    urdf_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", "my_robot.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", "robot_rviz_config.rviz"]
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
        # Node(
        #     package=pkg_name,
        #     executable='image_saver'
        # ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')])
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={'world': PathJoinSubstitution([FindPackageShare(pkg_name), 'worlds', 'updated_office.world'])}.items()
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-x', '1.0', '-y', '1.0', '-z', '0.0']
        )
    ])
