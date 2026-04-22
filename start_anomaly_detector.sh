#!/bin/bash
# Disaster Rescue Rover — Hazard & Survivor Detector
# Detects survivors, fire, hazmat, debris using camera + LiDAR
# Reports saved to ~/Documents/MAR_miniproject/reports/
#
# Options:
#   --show        : show live detection window
#   --conf 0.4    : confidence threshold (default 0.4)

source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

python3 /home/jalanth/Documents/MAR_miniproject/src/articubot_one/scripts/anomaly_detector.py --show $@
