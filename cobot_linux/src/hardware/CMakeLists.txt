cmake_minimum_required(VERSION 3.10)
project(cobot_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cobot_msgs REQUIRED)

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
ament_target_dependencies(cobot3_drive
  rclcpp std_msgs nav_msgs tf2 tf2_ros geometry_msgs sensor_msgs
)
target_link_libraries(cobot3_drive shared_libraries)

ament_package()
