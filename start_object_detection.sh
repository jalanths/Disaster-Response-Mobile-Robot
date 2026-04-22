#!/bin/bash
# Object Detection using YOLOv8 on robot camera feed
# Options:
#   --show         : show live detection window
#   --conf 0.5     : confidence threshold (default 0.5)
#   --model yolov8n.pt : YOLO model (n=nano, s=small, m=medium)

source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

python3 /home/jalanth/Documents/MAR_miniproject/src/articubot_one/scripts/object_detector.py --show $@
