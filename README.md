# 北航Transistor战队2025哨兵工作空间

> 写在前面
> 
> 此为新版README.md，在将北极熊大量功能包全部fork过来之后笔者重新编写了安装文档。

<div style="text-align: center;">
  <img src="images/transistor_code.png" width="400" height="400">
</div>

本仓库为北航Transistor战队2026赛季哨兵机器人的工作空间，参考了深北莫北极熊战队开源的导航功能包，丰富了假裁判系统等功能，同时做了一些修改。

本文档旨在帮助导航小白快速安装2025Transistor哨兵工作空间。

## 0. 前言

- Ubuntu 22.04
- ROS Humble

## 1. 拉取工作空间的远端仓库并安装子模块

```bash
git clone https://github.com/LemperorD/nav2_ws.git
```
```bash
git submodule update --init --recursive
```

## 2. 下载部分第三方软件

笔者将大部分需要在工作空间编译的第三方软件放在了``.gitignore``里防止更新时打扰到他们，所以此处需要自行使用vcstool,通过功能包中的``dependencies.repos``来拉取相应的第三方软件。

先配置相关环境。

```bash
sudo pip install vcstool2
```
```bash
pip install xmacro
```

然后正式拉取第三方软件。
```bash
vcs import src < dependencies.repos
```

如果您的ubuntu没有配置英伟达驱动，那么仿真将奇卡无比，**请移步飞书文档中的算法通用配置显卡驱动与英伟达容器工具箱**。

## 3. 先验点云地图

先验点云用于``small_gicp_relocalization``功能包重定位，由于点云文件体积较大，故不存储在 git 中，请前往[FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS)下载。

然后将下载目录中的四个点云文件（即``*.pcd``文件）移动到``~/nav2_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation``中。

## 4. small_gicp第三方库系统安装

``small_gicp_relocalization``功能包依赖于``small_gicp``这一第三方库进行开发。笔者习惯于将系统安装的第三方库放在主文件夹的自定义文件夹下编译安装，接下来的命令将如此做。

```bash
mkdir -p ~/tools && cd ~/tools
```
```bash
sudo apt install -y libeigen3-dev libomp-dev
```
```bash
git clone https://github.com/koide3/small_gicp.git
```
```bash
cd small_gicp
```
```bash
mkdir build && cd build
```
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
sudo make install
```

## 5. KISS_MatcherR第三方库系统安装

``buaa_sentry_relocalization``功能包的粗配准部分以及``kiss_matcher_relocalization``功能包依赖于``KISS_Matcher``这一第三方库进行开发。

```bash
mkdir -p ~/tools && cd ~/tools
```
```bash
git clone https://github.com/MIT-SPARK/KISS-Matcher.git
```
```bash
cd KISS-Matcher
```

修改Makefile中的cppinstall和cppinstall_matcher_only为如下所示（添加两行）
```
cppinstall: deps
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_FLAGS="-fPIC"
	@cmake --build cpp/kiss_matcher/build -j$(nproc --all)
	@$(SUDO) cmake --install cpp/kiss_matcher/build
	@$(SUDO) cmake --install cpp/kiss_matcher/build/_deps/robin-build

cppinstall_matcher_only:
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_CXX_FLAGS="-fPIC"
	@cmake --build cpp/kiss_matcher/build -j$(nproc --all)
	@$(SUDO) cmake --install cpp/kiss_matcher/build
```

```bash
make deps & make cppinstall
```

## 6. TEASER-plusplus第三方库系统安装

```bash
sudo apt install cmake libeigen3-dev libboost-all-dev
mkdir -p ~/tools && cd ~/tools
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. && make
sudo make install
sudo ldconfig
```

## 7. 编译

如果安装了MVS，请在**编译**和**运行导航**前运行

```bash
export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib
```

上述步骤已经将所需的都安装好了，可以开始编译了。

```bash
cd ~/nav2_ws
```
```bash
rosdep init # 如果初始化过rosdep请忽略这一步
```
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel 2
```

## 8. 运行

先在工作空间根目录下source环境变量。
```bash
source ./install/setup.bash
```

### terminal 1

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

此处记得**点击gazebo界面中左下角的启动键**，不然将无法发布机器人内部各个零件间的tf变换。

### terminal 2

如果rviz出现闪烁问题，可以尝试先设置环境变量
```bash
export QT_SCREEN_SCALE_FACTORS=1
```
或
```bash
export QT_ENABLE_HIGHDPI_SCALING=0
```

#### 导航模式

```bash
source ./install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmul_2025 \
slam:=False
#注：上述命令使用了联盟赛的3v3地图，方便后续行为树决策部分使用
```

#### 建图模式

```bash
source ./install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

具体的更改地图、调参等功能请移步``tutorial.md``查看。

## 9. 假裁判系统及操作手客户端

假裁判系统及操作手客户端需安装如下python第三方库
```bash
pip install engineio
pip install Flask
pip install flask-cors
pip install flask-socketio
```
使用``local_referee_2350.sh``在``localhost:2350``打开网页端假裁判系统

使用``local_player_5000.sh``在``localhost:5000``打开网页端假操作手客户端
