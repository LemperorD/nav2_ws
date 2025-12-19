#!/bin/bash
source ./install/setup.bash

cd ~/nav2_ws/map_reality || {
  echo "目录 map_reality 不存在!"
  exit 1
}

TIME_STR=$(date +"%Y-%m-%d_%H-%M-%S")
MAP_NAME="map_real_${TIME_STR}"

echo "保存 2D 栅格地图到 $(pwd)/${MAP_NAME}.*"
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"
