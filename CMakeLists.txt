cmake_minimum_required(VERSION 3.8)
project(joy_to_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)        
find_package(sensor_msgs REQUIRED)  
find_package(geometry_msgs REQUIRED) 

add_executable(joy_to_twist_node src/joy_to_twist_node.cpp)
add_executable(joy_to_twist_sub src/joy_to_twist_sub.cpp)

ament_target_dependencies(joy_to_twist_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)
ament_target_dependencies(joy_to_twist_sub 
  rclcpp 
  sensor_msgs 
  geometry_msgs
)

install(TARGETS joy_to_twist_node joy_to_twist_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()