cmake_minimum_required(VERSION 3.10)
project(cobot_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cobot_msgs REQUIRED)

# External/system libraries
find_library(POPT_LIB popt REQUIRED)
find_library(GLOG_LIB glog REQUIRED)

# Group dependencies
set(ROS2_DEPENDENCIES
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  cobot_msgs
)

set(SYSTEM_LIBS
  ${POPT_LIB}
  ${GLOG_LIB}
  shared_libraries
)

# Include shared headers
include_directories(
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/..
  ${PROJECT_SOURCE_DIR}/../shared/util
  ${PROJECT_SOURCE_DIR}/../shared/math
)

# Source files for cobot3_drive
set(DRIVE_COMMON_SRCS
  drive.cpp
  fcs.cpp
  serial.cc
)

set(COBOT3_SRCS
  cobot3_drive/cobot_drive_main.cpp
  ${DRIVE_COMMON_SRCS}
)

# Build cobot3_drive executable
add_executable(cobot3_drive ${COBOT3_SRCS})
ament_target_dependencies(cobot3_drive ${ROS2_DEPENDENCIES})
target_link_libraries(cobot3_drive ${SYSTEM_LIBS})

ament_package()
