# 北航Transistor战队2026赛季哨兵底盘通信部分

## 0. 前言
本赛季上位机与C板的通信部分使用了C++模板类以提升代码复用率，详情可见``include/ros_serial_bridge.hpp``。

功能包代码架构如下：
```bash
├── CMakeLists.txt
├── config
│   └── communication.yaml # 参数文件，可以自定义串口位置与波特率，默认为/dev/ttyACM0 & B115200
├── include
│   └── communication
│       ├── bridge.hpp # 通信节点类定义
│       ├── Com.h # 串口类定义
│       └── ros_serial_bridge.hpp # 通信模板类
├── launch
│   └── communication.launch.py
├── package.xml
├── README.md
└── src
    ├── bridge.cpp
    ├── cmdvel_sine_pub.cpp # 测试/cmd_vel消息通信使用的脚本，以正弦信号发布速度消息
    └── Com.cpp
```

下面简单阐述如何使用模板类增加通信的消息种类。

## 1. 如何使用模板类增加通信的消息种类

### 1.1 在串口类Com.h增加对应数据帧长度的缓冲区并增加对应的receive函数（接收下位机信息时使用）

通信模板类调用的接收串口数据的函数默认没有参数并返回数组指针，需要串口类内部调用相关的全局变量，所以要在串口类定义的时候定义相关的全局变量。

### 1.2 在通信节点类bridge.hpp & bridge.cpp进行模板类的定义与实现

写好自己要解码或编码的ROS消息类型以及对应的编解码函数，实现通信模板类

### 1.3 完善消息类型相关的头文件以及CMakeLists.txt & package.xml

