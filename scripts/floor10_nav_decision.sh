# 北航机器人队十楼实验室导航测试脚本
cd ~/nav2_ws
source install/setup.bash
 ros2 launch pb2025_nav_bringup rm_navigation_reality_launch_nm.py world:=floor10 slam:=True behavior_tree_type:=decision_simple
#ros2 launch pb2025_nav_bringup rm_navigation_reality_launch_nm.py world:=floor10 slam:=True behavior_tree_type:=pb2025_sentry_behavior