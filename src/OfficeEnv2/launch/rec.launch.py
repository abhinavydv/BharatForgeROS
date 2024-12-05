from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

pkg_name = "OfficeEnv2"

def generate_launch_description():
    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),

        Node(
            package=pkg_name,
            executable='recogniser'
        ),
        
    ])