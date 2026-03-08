#!/bin/bash
# 北航机器人队十楼实验室导航测试脚本（带行为树选择）
# 切换到工作区并 source 环境
cd ~/nav2_ws
source install/setup.bash

# --- 默认参数 ---
BEHAVIOR_TREE=${1:-decision_simple}   # 默认使用 decision_simple

# --- 启动导航 ---
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch_nm.py world:=floor10 slam:=False

# --- 启动选择 ---
if [ "$BEHAVIOR_TREE" = "decision_simple" ]; then
    echo "启动 decision_simple 行为树..."
    ros2 launch decision_simple decision_simple.launch.py
elif [ "$BEHAVIOR_TREE" = "pb2025_sentry_behavior" ]; then
    echo "启动 pb2025_sentry_behavior 行为树..."
    ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
else
    echo "未知行为树: $BEHAVIOR_TREE"
    exit 1
fi