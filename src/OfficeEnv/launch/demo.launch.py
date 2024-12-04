
# Example to run rgbd datasets:
# [ROS1] Prepare ROS1 rosbag for conversion to ROS2
#   $ wget http://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.bag
#   $ rosbag decompress rgbd_dataset_freiburg3_long_office_household.bag
#   $ wget https://raw.githubusercontent.com/srv/srv_tools/kinetic/bag_tools/scripts/change_frame_id.py
#   Edit change_frame_id.py, remove/comment lines beginning with "PKG" and "import roslib", change line "Exception, e" to "Exception"
#   $ roscore
#   $ python3 change_frame_id.py -o rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag -i rgbd_dataset_freiburg3_long_office_household.bag -f openni_rgb_optical_frame -t /camera/rgb/image_color
# [ROS2]
#   $ sudo pip install rosbags     # See https://docs.openvins.com/dev-ros1-to-ros2.html
#   $ rosbags-convert rgbd_dataset_freiburg3_long_office_household_frameid_fixed.bag

#   $ ros2 launch rtabmap_examples rgbdslam_datasets.launch.py
#   $ cd rgbd_dataset_freiburg3_long_office_household_frameid_fixed
#   $ ros2 bag play rgbd_dataset_freiburg3_long_office_household_frameid_fixed.db3 --clock


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap
from launch_ros.actions import SetParameter
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

NUM_ROBOTS = 1

def generate_launch_description():

    nodes = []
    
    for i in range(NUM_ROBOTS):
        robot = f'robot_{i}'

        parameters=[{
            'frame_id':f'{robot}/kinect',
            'ground_truth_frame_id': '/world',
            'subscribe_depth':True,
            'subscribe_odom_info':True,
            'topic_queue_size':20,
            'sync_queue_size':20,
            'approx_sync_max_interval':0.25,
            # 'subscribe_rgbd':True,

            # RTAB-Map's parameters should all be string type:
            'Odom/Strategy':'1', # F to M
            'Odom/ResetCountdown':'30',
            'Odom/GuessSmoothingDelay':'0',
            'Rtabmap/StartNewMapOnLoopClosure':'true',
            'RGBD/CreateOccupancyGrid':'true',
            'Rtabmap/CreateIntermediateNodes':'true',
            'RGBD/LinearUpdate':'0',
            'RGBD/AngularUpdate':'0'
            }]

        remappings=[
            ('rgb/image', f'{robot}/camera/image_raw'),
            ('rgb/camera_info', f'{robot}/camera/camera_info'),
            ('depth/image', f'{robot}/camera/depth/image_raw'),
        ]

        nodes.append(Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),)

        nodes.append(Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),)

        nodes.append(Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),)

        # /tf topic is missing in the converted ROS2 bag, create a fake tf
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.0', '0.0', '0.0', '-1.57079632679', '0.0', '-1.57079632679', f'{robot}/kinect', f'{robot}/camera_link_optical']
        ),)
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen', 
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', f'{robot}/kinect', f'{robot}/base_link']
        ),)


    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above

        # Nodes to launch
        *nodes
    ])
