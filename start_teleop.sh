#!/bin/bash
# Terminal 4: Keyboard control
source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
