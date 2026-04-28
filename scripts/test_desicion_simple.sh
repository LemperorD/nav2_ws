#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd /home/rog/nav2_ws

echo "Building decision_simple..."
if ! colcon build --packages-select decision_simple --cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON; then
  echo "Build failed!"
  cd /home/rog/nav2_ws
  exit 0
fi

echo "Build successful, running tests..."
. /home/rog/nav2_ws/install/setup.bash
cd /home/rog/nav2_ws/build/decision_simple && ./test_general

cd /home/rog/nav2_ws