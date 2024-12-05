template = """
use_sim_time: true  # Set to true if using simulated time
slam_mode: true      # Online SLAM mode
map_file_name: ""
resolution: 0.05
publish_map: true
map_update_interval: 2.0
use_scan_matching: true
use_scan_barycenter: false
transform_tolerance: 0.1
scan_topic: {{ robot_name }}/scan
odom_topic: {{ robot_name }}/odom
max_laser_range: 30.0
minimum_time_interval: 0.0
mode: "mapping"  # or "localization" if you're localizing
"""

NUM_ROBOTS = 15

for i in range(NUM_ROBOTS):
    with open(f"slam_toolbox_configs/slam_toolbox_config_{i}.yaml", "w") as f:
        f.write(template.replace("{{ robot_name }}", f"robot_{i}"))
