#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/nav2_ws

echo "Building decision_simple..."
colcon build --packages-select decision_simple --cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
. ~/nav2_ws/install/setup.bash
cd ~/nav2_ws/build/decision_simple && ./test_general
cd ~/nav2_ws
