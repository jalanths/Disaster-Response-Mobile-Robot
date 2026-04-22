# Disaster-Response Mobile Robot

A ROS 2 autonomous mobile robot for disaster-zone response, built on Gazebo simulation with SLAM mapping, Nav2 waypoint navigation, YOLOv8 object detection, anomaly detection, battery monitoring, and a live web dashboard.

## Features

- **SLAM & Localization** — online async SLAM (slam_toolbox) and AMCL localization against a pre-built disaster/warehouse map
- **Autonomous Navigation** — Nav2 stack with waypoint missions (e.g. `search_sweep`)
- **Object Detection** — YOLOv8 (`yolov8n.pt`) on the robot's camera feed
- **Anomaly Detection** — flags unexpected scene content with configurable confidence threshold
- **Battery Simulator** — publishes simulated battery state for mission-planning logic
- **Dynamic Obstacles** — moving obstacles injected at runtime to stress-test navigation
- **Web Dashboard** — live telemetry and camera view in the browser
- **RViz & Gazebo** — visualisation and simulation out of the box

## Repository Layout

```
.
├── src/articubot_one/        # ROS 2 package (description, launch, config, scripts)
│   ├── launch/               # sim, nav, localization, SLAM, camera, lidar launch files
│   └── scripts/              # anomaly_detector, battery_simulator, object_detector,
│                             # waypoint_navigator, web_dashboard
├── launch_all.sh             # brings up the full stack in separate terminals
├── kill_all.sh               # tears everything down
├── start_*.sh                # individual component launchers
├── disaster_map.{pgm,yaml}   # pre-built disaster map
├── map_republish.py          # republish map on /map topic
└── save_map.sh               # persist the current SLAM map
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Nav2, slam_toolbox, AMCL
- Python 3.10+ with `ultralytics` (YOLOv8), `opencv-python`, `flask` (dashboard)

## Build

```bash
cd ~/Documents/MAR_miniproject
colcon build --symlink-install
source install/setup.bash
```

## Run

Full stack (opens ~10 terminals):

```bash
bash launch_all.sh
```

Individual components:

```bash
bash start_sim.sh              # Gazebo simulation
bash start_map.sh              # map server
bash start_rviz.sh             # RViz
bash start_localization.sh     # AMCL
bash start_nav2.sh             # Nav2
bash start_slam.sh             # SLAM (alternative to map+AMCL)
bash start_object_detection.sh # YOLOv8
bash start_anomaly_detector.sh --show --conf 0.2
bash start_dashboard.sh        # web dashboard
bash start_dynamic_obstacles.sh
bash start_waypoints.sh --mission search_sweep
bash start_teleop.sh           # keyboard teleop
```

Shut everything down:

```bash
bash kill_all.sh
```

## Saving Maps

After SLAM, save the current map:

```bash
bash save_map.sh
```

## Notes

- `yolov8n.pt` is not tracked in git — download it from [Ultralytics](https://github.com/ultralytics/assets/releases) or the package will fetch it on first run.
- SLAM serialized data (`mymapserial.*`) and `build/`, `install/`, `log/` are ignored by git.
