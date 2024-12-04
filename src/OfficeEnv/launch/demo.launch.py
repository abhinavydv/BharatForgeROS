
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

def generate_launch_description():

    pkg_name = 'OfficeEnv'

    urdf_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "urdf", "my_robot.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare(pkg_name), "rviz", "robot_rviz_config.rviz"]
    )

    parameters=[{
          'frame_id':'kinect',
          #'ground_truth_frame_id': '/map',
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
          'RGBD/CreateOccupancyGrid':'false',
          'Rtabmap/CreateIntermediateNodes':'true',
          'RGBD/LinearUpdate':'1',
          'RGBD/AngularUpdate':'1'
          }]
          
    remappings=[
          ('rgb/image', '/camera/rgb/image_color'),
          ('rgb/camera_info', '/camera/rgb/camera_info'),
          ('depth/image', '/camera/depth/image')
    ]


    return LaunchDescription([

        SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
       
        # /tf topic is missing in the converted ROS2 bag, create a fake tf
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=['0.0', '0.0', '0.0', '-1.57079632679', '0.0', '-1.57079632679', 'kinect', 'camera_link_optical']
        ),
        Node( 
            package='tf2_ros', executable='static_transform_publisher', output='screen', 
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'kinect', 'base_link']
        ),
        Node( 
             package='tf2_ros', executable='static_transform_publisher', output='screen', 
             arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link_optical', 'base_link']
        )
        # '''
        # DeclareLaunchArgument(
        #     'urdf_path',
        #     default_value=urdf_path,
        #     description='Path to the URDF file'
        # ),
        # DeclareLaunchArgument(
        #     'rviz_config_path',
        #     default_value=rviz_config_path,
        #     description='Path to the RViz config file'
        # ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     output='screen',
        #     arguments=['-d', LaunchConfiguration('rviz_config_path')]
        # ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     parameters=[{
        #         'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')])
        #     }]
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        #     ]),
        #     launch_arguments={'world': PathJoinSubstitution([FindPackageShare(pkg_name), 'worlds', 'updated_office.world'])}.items()
        # ),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=['-topic', 'robot_description', '-entity', 'my_robot', '-x', '1.0', '-y', '1.0', '-z', '0.0']
        # )
        # '''
    ])
