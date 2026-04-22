#!/bin/bash
# Terminal 1: Kill stale Gazebo and launch simulation
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f gzclient 2>/dev/null
sleep 2

source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=/opt/ros/humble/share/aws_robomaker_small_warehouse_world/models:$GAZEBO_MODEL_PATH
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

cd /home/jalanth/Documents/MAR_miniproject
ros2 launch articubot_one launch_sim.launch.py world:=./src/articubot_one/worlds/disaster.world
