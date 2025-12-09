cmake_minimum_required(VERSION 3.8)
project(joy_to_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

# Joy To Twist (Publisher)
add_executable(joy_to_twist_node
  src/joy_to_twist_node_main.cpp 
  src/joy_to_twist_node.cpp  
)
target_include_directories(joy_to_twist_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(joy_to_twist_node rclcpp sensor_msgs geometry_msgs)

# Joy To Twist Sub (Logger)
add_executable(joy_to_twist_sub
  src/joy_to_twist_sub_main.cpp 
  src/joy_to_twist_sub.cpp
)
target_include_directories(joy_to_twist_sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(joy_to_twist_sub rclcpp sensor_msgs geometry_msgs)

# Joy To Twist TUI (Interfejs)
# (Interface)
add_executable(joy_to_twist_tui 
  src/joy_to_twist_tui_main.cpp 
  src/joy_to_twist_tui.cpp
)
target_include_directories(joy_to_twist_tui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(joy_to_twist_tui rclcpp sensor_msgs geometry_msgs)

# Instalacja
# Installation

install(TARGETS 
  joy_to_twist_node
  joy_to_twist_sub
  joy_to_twist_tui
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()