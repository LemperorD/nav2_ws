## 1. 项目简介
本项目是基于C++编写的简易阶乘计算程序，采用**头文件和源文件分离**的标准开发，使用CMake进行项目编译管理。

## 2. 项目目录结构
```tree
├── CMakeLists.txt
├── include
│   └── factorial.h
├── README.md
└── src
    ├── factorial.cpp
    └── main.cpp
```

## 3. 运行环境
系统：Linux/macOS
依赖工具：C++编译器/CMake/make

## 4.使用教程
```bash 
#创建编译目录
mkdir build && cd build 
#生成编译文件
camke ..
# 编译项目
make
#启动程序
./factorial
```