# 北极熊导航Tutorial

<div style="text-align: center;">
  <img src="images/transistor_code.png" width="400" height="400">
</div>

本教程旨在让**已经实践过Quick-Start流程**的同学学会如何调参，同时通过北极熊开源的导航功能包更加深入地理解建图、定位、导航、决策的原理和四者密不可分的关系。

本教程将长期更新。

## 0. 前言——北极熊导航包功能补全

Quick-Start中涉及到的功能包仅仅包含了建图、定位与导航的功能，不涉及任何决策，并不是北极熊导航仿真所使用的全部功能包，所以要进行补全。

### (1). 一口气全克隆

在主文件夹（``cd ~``）中执行如下指令：
```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws.git
```
```bash
cd pb2025_sentry_ws
```
```bash
vcs import --recursive . < dependencies.repos
```

上述部分将克隆北极熊战队25赛季哨兵的工作空间，其规范的文件管理让笔者叹为观止。但如果只是在自己的电脑里跑仿真，那该工作空间中``standard_robot_pp_ros2``（用来与下位机通信的）是不需要的。可以将其手动剔除或按照后续方式对快速开始部分使用的功能包补全。

北极熊导航的视觉部分提供了OpenVINO版本，若想安装请移步如下链接：

[Install OpenVINO](https://docs.openvino.ai/2025/get-started/install-openvino.html?PACKAGE=OPENVINO_BASE&VERSION=v_2025_2_0&OP_SYSTEM=LINUX&DISTRIBUTION=PIP)

上述命令建立在您使用过Quick-Start并创建了相应环境变量，如果您还在新建文件夹，建议移步下述链接看官方教程：

[More about pb2025_sentry_ws](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_ws)

### (2). 行为树

```bash
cd ~/nav2_ws
```
```bash
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior.git src/pb2025_sentry_behavior
```
```bash
vcs import --recursive src < src/pb2025_sentry_behavior/dependencies.repos
```
```bash
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
```

上述步骤将下载并编译安装北极熊导航仿真中的行为树部分及相关依赖，有关更多信息请移步如下链接。

[More about pb2025_sentry_behavior](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_behavior)

### (3). 视觉

```bash
cd ~/nav2_ws
```
```bash
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision.git src/pb2025_rm_vision
```
```bash
vcs import --recursive src < src/pb2025_rm_vision/dependencies.repos
```
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

上述步骤将下载并编译安装北极熊导航仿真中的视觉部分及相关依赖，有关更多信息请移步如下链接。

[More about pb2025_rm_vision](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision)

## 1. 简略调参教程

接下来将进入正式的tutorial部分。本章为简略调参教程，不涉及每个功能包的细致调参，聚焦的是更改地图、更改仿真建模等更重要、更核心的参数。

### 1.1 更改仿真地图

#### 1.1.1 gazebo端

经历过Quick-Start流程的你肯定知道，gazebo端指的是``rmu_gazebo_simulator``，在该功能包下找到``config/gz_world.yaml``

```yaml
world: "rmul_2025" # 在此处修改地图

robots:
..... # 可以更改机器人的命名空间、出生点等
```

#### 1.1.2 导航端

经历过Quick-Start流程的你肯定知道，导航端改变地图是为了更改导航模式下的先验点云地图与先验二维栅格地图。我们先不定位某一个功能包，先说最简单的修改方式：在launch命令里修改。

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=False\
world:=rmul_2025 # world参数这一行就可以修改了
```

#### 1.1.3 相关文件到底存在哪里

经历过Quick-Start流程的你肯定知道，建图仿真可以保存自己建出来的点云图和二维栅格地图。实际赛场上我们很可能也要根据赛场情况，在准备时间里现场建图，那我们就很需要知道地图都保存在了哪里。

##### a. gazebo仿真世界地图

设想一下，不久之后RM官方为了各个战队调车方便将26赛季新地图发了出来，我们就很需要知道gazebo仿真世界地图在哪里，方便我们下载好放到相应文件夹里实现新赛季的赛博调车。

其位置在``rmu_gazebo_simulator``中的``resource/model``和``resource/world``里。

##### b. 先验点云和先验二维栅格地图

- 先验点云：``pb2025_nav_bringup``功能包中的``pcd``文件夹
- 先验栅格：``pb2025_nav_bringup``功能包中的``map``文件夹

至于保存在``reality``还是``simulation``里就要看实际需求了。

### 1.2 更改仿真中雷达的位姿

#### 1.2.1 在哪里改

位置在``pb2025_robot_description``功能包的``resource/xmacro/simulation_robot.sdf.xmaro``，主要关注下面这一句：

```xml
<!--livox-->
<xmacro_block name="livox" prefix="front_" parent="chassis" pose="0.16 0.0 0.18 ${pi/6} 0.0 ${pi/2}" update_rate="20" samples="1875"/>
```
其中``pose``属性中六个浮点数分别为``x, y, z, roll, pitch, yaw``，符合ROS2中的定义，可以通过更改``pose``属性来达到在仿真中更改雷达位姿的目的。

如果只是更改雷达的**位置**那还好说，但如果要更改雷达的**姿态（即对欧拉角进行更改），则还有一处很重要的参数要改！**

#### 1.2.2 Point-LIO重力相关问题

[Point-LIO](https://github.com/hku-mars/Point-LIO)，是由伟大的[HKU-Mars-Lab](https://github.com/hku-mars)开源的一款功能十分强大的激光雷达惯性里程计（Lidar-Inertial Odometry，以下简称LIO）。

此处先不将与Point-LIO相关的内容陈述殆尽，我们只需要知道，Point-LIO发出的里程计坐标系是**激光雷达在线对准重力的结果**。

稍微细致一点解释一下，如果Point-LIO程序开启时通过输入的IMU信息检测到雷达没有正常放置，那Point-LIO会自动对准重力，保证里程计系的z轴与重力加速度g在同一直线上。

所以理论上，在不进行调参的情况下，如果雷达倾斜放置（对于mid360，这是个很常见的情况），那么lidar_odom与lidar两个坐标系在程序上电时并不是完全对准的，这会对后续的``loam_interface``造成影响。

那我们如何修改参数呢？答案是**Point-LIO在线对准重力使用的是相应参数文件中的一个参数**，把这个重力也按照你的雷达倾转一定方向就好了。

经历过Quick-Start流程的你肯定知道，仿真流程中除了与gazebo直接相关的部分，其余参数都在``pb2025_nav_bringup``功能包中的``config/simulation/nav2_params.yaml``中，找到对应功能包的参数就好啦。

```yaml
point_lio:
  ros__parameters:

    ...... # 省略

    mapping:
      imu_en: True
      extrinsic_est_en: False                           # for aggressive motion, set this variable False
      imu_time_inte: 0.005                              # = 1 / frequency of IMU
      lidar_time_inte: 0.1
      satu_acc: 30.0                                    # the saturation value of IMU's acceleration. not related to the units
      satu_gyro: 35.0                                   # the saturation value of IMU's angular velocity. not related to the units
      acc_norm: 9.81                                    # 1.0 for g as unit, 9.81 for m/s^2 as unit of the IMU's acceleration
      lidar_meas_cov: 0.001                             # 0.001
      acc_cov_output: 500.0
      gyr_cov_output: 1000.0
      b_acc_cov: 0.0001
      b_gyr_cov: 0.0001
      imu_meas_acc_cov: 0.1
      imu_meas_omg_cov: 0.1
      gyr_cov_input: 0.01                               # for IMU as input model
      acc_cov_input: 0.1                                # for IMU as input model
      plane_thr: 0.1                                    # 0.05, the threshold for plane criteria, the smaller, the flatter a plane
      match_s: 81.0
      ivox_grid_resolution: 0.5
      gravity: [ 0.0, -4.9, -8.487047153776548 ]        # gravity to be aligned # rpy = [0, pi/6, 0]
      gravity_init: [ 0.0, -4.9, -8.487047153776548 ]   # preknown gravity in the first IMU body frame, use when imu_en is False or start from a non-stationary state
      extrinsic_T: [ 0.0, 0.0, 0.0 ]
      extrinsic_R: [ 1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0 ]
```

其中较为重要的是``acc_norm``和``gravity``这两个参数，这要根据IMU数据的实际情况调整。

- acc_norm：加速度范数，如果IMU返回标准重力加速度的值为1则设成1,否则为9.81
- gravity：当地重力加速度在IMU三轴的分量，根据实际姿态调整