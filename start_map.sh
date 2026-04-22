#!/bin/bash
# Terminal 2: Launch map server with saved map
source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

cd /home/jalanth/Documents/MAR_miniproject
ros2 launch articubot_one map_server.launch.py
