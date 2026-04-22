#!/bin/bash
# AMCL only — map_server is launched separately via start_map.sh
source /opt/ros/humble/setup.bash
source /home/jalanth/Documents/MAR_miniproject/install/setup.bash

cd /home/jalanth/Documents/MAR_miniproject

ros2 run nav2_amcl amcl --ros-args \
  --params-file /home/jalanth/Documents/MAR_miniproject/src/articubot_one/config/nav2_params.yaml \
  -p use_sim_time:=True &

AMCL_PID=$!

# Wait until map_server has a publisher on /map
echo "Waiting for /map topic..."
until ros2 topic info /map 2>/dev/null | grep -q "Publisher count: [1-9]"; do
  sleep 1
done
echo "/map is available."
sleep 2

source /opt/ros/humble/setup.bash
ros2 lifecycle set /amcl configure
ros2 lifecycle set /amcl activate

echo "AMCL is active."
wait $AMCL_PID
