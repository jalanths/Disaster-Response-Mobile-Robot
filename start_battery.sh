#!/bin/bash
# Battery Simulator — drains 0.15%/sec (~11 min full patrol)
# Publishes: /battery_level (Float32), /battery_status (String: OK/LOW/CRITICAL)
source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash
python3 /home/jalanth/Documents/MAR_miniproject/src/articubot_one/scripts/battery_simulator.py
