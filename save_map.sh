#!/bin/bash
# Save the current SLAM map to disk
# Run this while SLAM is active after exploring the environment
source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

MAP_NAME=${1:-mymapsave}
MAP_PATH=/home/jalanth/Documents/MAR_miniproject/$MAP_NAME

echo "Saving map to $MAP_PATH ..."
ros2 run nav2_map_server map_saver_cli -f $MAP_PATH --ros-args -p use_sim_time:=true
echo "Map saved: ${MAP_PATH}.pgm and ${MAP_PATH}.yaml"
