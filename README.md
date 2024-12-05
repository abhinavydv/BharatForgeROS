# BharatForgeROS

## How to run the simulation
Ensure you have ROS humble and Gazebo 11 installed
- unzip team_39.zip
- cd code
- apt install ros-humble-cartographer-ros
- pip install ultralytics gymnasium matplotlib ray scipy numpy==1.23.0
- colcon build
- ros2 launch OfficeEnv2 depth.launch.py
- ros2 launch OfficeEnv2 cartographer.launch.py
- ros2 launch OfficeEnv2 rec.launch.py

## Folder structure
- Root Folder
    - code
        - src
            - OfficeEnv2
                - launch
                - OfficeEnv2
                - resource
                - rviz
                - test
                - urdf
                - worlds
                - package.xml
                - setup.py
        - README.md
    - videos
    - README
    - report.pdf

