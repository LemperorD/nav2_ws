cmake_minimum_required(VERSION 3.5)
project(communication)

# C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # 新增

# 库
add_library(communication SHARED src/Com.cpp)
target_include_directories(communication PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(communication rclcpp)

# 可执行
add_executable(array25_bridge src/array25_bridge.cpp)
target_link_libraries(array25_bridge communication)
ament_target_dependencies(array25_bridge
  rclcpp
  std_msgs
  geometry_msgs  # 新增
)

# 关键：不要有任何把 src/ 整目录装进 lib 的 install(DIRECTORY ...)！
# 正确的安装规则只有这一段：
install(TARGETS communication array25_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
