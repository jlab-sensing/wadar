cmake_minimum_required(VERSION 3.5)
project(chipotle-radar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(inertial_sense_ros2 REQUIRED)    

# add_dependencies(chipotle-radar ${catkin_EXPORTED_TARGETS})

# include_directories(include)
# include_directories(${CMAKE_SOURCE_DIR}/../../signal_processing)

add_executable(gps_scan_node 
src/gps_scan_node.cpp
)
ament_target_dependencies(gps_scan_node rclcpp std_msgs geometry_msgs inertial_sense_ros2)  

install(TARGETS gps_scan_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
