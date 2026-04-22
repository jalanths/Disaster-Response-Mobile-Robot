#!/bin/bash
# SLAM — builds a new map in real time while you drive
# Drive the robot with teleop to explore the environment
# Save the map when done: bash save_map.sh

source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

cd /home/jalanth/Documents/MAR_miniproject
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=/home/jalanth/Documents/MAR_miniproject/src/articubot_one/config/mapper_params_mapping.yaml \
  use_sim_time:=true
