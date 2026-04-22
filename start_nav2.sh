#!/bin/bash
# Launch Nav2 navigation stack with autostart
source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

cd /home/jalanth/Documents/MAR_miniproject
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=True \
  autostart:=True \
  params_file:=/home/jalanth/Documents/MAR_miniproject/src/articubot_one/config/nav2_params.yaml

