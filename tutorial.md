# 北极熊导航Tutorial

## 0. 前言——北极熊导航包功能补全

Quick-Start中涉及到的功能包并不是北极熊导航仿真所使用的全部功能包，所以要进行补全。

### 1. 行为树

```bash
cd ~/nav2_ws
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior.git src/pb2025_sentry_behavior
vcs import --recursive src < src/pb2025_sentry_behavior/dependencies.repos
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

上述步骤将下载并编译安装北极熊导航仿真中的行为树部分及相关依赖，有关更多信息更多信息请移步如下链接。

[More about pb2025_sentry_behavior](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior)

### 2. 视觉



## 1. 如何更改仿真地图

## 2. 如何使用其他行为树