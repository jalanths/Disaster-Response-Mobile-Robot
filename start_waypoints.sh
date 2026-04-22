#!/bin/bash
# Disaster Rescue Rover — Waypoint Navigator
#
# List all locations and missions:
#   bash start_waypoints.sh --list
#
# Go to a single location:
#   bash start_waypoints.sh --go base_camp
#   bash start_waypoints.sh --go search_zone_a
#
# Run a rescue mission:
#   bash start_waypoints.sh --mission search_sweep
#   bash start_waypoints.sh --mission full_coverage --loop
#   bash start_waypoints.sh --mission rescue_run
#   bash start_waypoints.sh --mission hazard_check

source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

python3 /home/jalanth/Documents/MAR_miniproject/src/articubot_one/scripts/waypoint_navigator.py $@
