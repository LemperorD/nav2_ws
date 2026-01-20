cmake_minimum_required(VERSION 3.8)
project(decision_simple)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pb_rm_interfaces REQUIRED)

# auto_aim_interfaces 可能没有：可选
find_package(auto_aim_interfaces QUIET)
if(auto_aim_interfaces_FOUND)
  add_compile_definitions(DECISION_SIMPLE_HAS_AUTO_AIM)
endif()

include_directories(include)

add_library(decision_simple_component SHARED
  src/decision_simple.cpp
)
ament_target_dependencies(decision_simple_component
  rclcpp rclcpp_components
  std_msgs geometry_msgs nav_msgs
  tf2 tf2_geometry_msgs
  pb_rm_interfaces
  nav2_msgs
)
if(auto_aim_interfaces_FOUND)
  ament_target_dependencies(decision_simple_component auto_aim_interfaces)
endif()

rclcpp_components_register_nodes(decision_simple_component "decision_simple::DecisionSimple")

add_executable(decision_simple_node src/decision_simple_main.cpp)
target_link_libraries(decision_simple_node decision_simple_component)
ament_target_dependencies(decision_simple_node rclcpp)

install(TARGETS
  decision_simple_component
  decision_simple_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
