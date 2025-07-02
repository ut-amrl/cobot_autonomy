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

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set(COBOT3_SRC
  cobot3_drive/cobot_drive_main.cpp
)
add_executable(cobot3_drive ${COBOT3_SRC})

ament_target_dependencies(cobot3_drive
  rclcpp std_msgs nav_msgs tf2 tf2_ros geometry_msgs sensor_msgs
)

install(TARGETS cobot3_drive
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
