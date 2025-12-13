cmake_minimum_required(VERSION 3.5)
project(communication)

# C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# 库
add_library(communication SHARED src/Com.cpp)
target_include_directories(communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(communication rclcpp)

# 可执行
add_executable(bridge_node src/bridge.cpp)
target_link_libraries(bridge_node communication)
ament_target_dependencies(bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
)

add_executable(cmdvel_sine_pub src/cmdvel_sine_pub.cpp)
ament_target_dependencies(cmdvel_sine_pub rclcpp geometry_msgs)
install(TARGETS cmdvel_sine_pub DESTINATION lib/${PROJECT_NAME})

install(TARGETS communication bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
