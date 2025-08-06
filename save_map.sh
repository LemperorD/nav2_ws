#!/bin/bash

source ./install/setup.bash

cd map_simulation || {
  echo "目录 map_simulation 不存在！"
  exit 1
}

TIME_STR=$(date +"%Y-%m-%d_%H-%M-%S")
MAP_NAME="map_simu_${TIME_STR}"

ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" --ros-args -r __ns:=/red_standard_robot1
