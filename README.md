# BharatForgeROS

## How to run the simulation
Ensure you have ROS humble and Gazebo 11 installed
- unzip 39_m2_kalyani.zip
- cd 39_m2_kalyani/code
- apt install ros-humble-cartographer-ros
- pip install ultralytics gymnasium matplotlib ray scipy numpy==1.23.0
- colcon build
- source install/setup.bash
- ros2 launch OfficeEnv2 depth.launch.py
- ros2 launch OfficeEnv2 cartographer.launch.py
- ros2 launch OfficeEnv2 rec.launch.py

## Folder structure
- 39_m2_kalyani/code
    - README
    - report.pdf
    - videos
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

