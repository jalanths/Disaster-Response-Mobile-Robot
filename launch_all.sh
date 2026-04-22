#!/bin/bash
cd ~/Documents/MAR_miniproject

gnome-terminal --title="1 - Simulation" -- bash -c "bash start_sim.sh; exec bash"
sleep 5

gnome-terminal --title="2 - Map Server" -- bash -c "bash start_map.sh; exec bash"
sleep 3

gnome-terminal --title="3 - RViz" -- bash -c "bash start_rviz.sh; exec bash"
sleep 2

gnome-terminal --title="4 - AMCL" -- bash -c "sleep 8 && bash start_localization.sh; exec bash"
sleep 2

gnome-terminal --title="5 - Battery" -- bash -c "bash start_battery.sh; exec bash"
sleep 1

gnome-terminal --title="6 - Nav2" -- bash -c "bash start_nav2.sh; exec bash"
sleep 3

gnome-terminal --title="7 - Anomaly Detector" -- bash -c "bash start_anomaly_detector.sh --show --conf 0.2; exec bash"
sleep 1

gnome-terminal --title="8 - Web Dashboard" -- bash -c "bash start_dashboard.sh; exec bash"
sleep 1

gnome-terminal --title="9 - Dynamic Obstacles" -- bash -c "sleep 10 && bash start_dynamic_obstacles.sh; exec bash"
sleep 1

gnome-terminal --title="10 - Waypoints" -- bash -c "sleep 15 && bash start_waypoints.sh --mission search_sweep; exec bash"
