cmake_minimum_required(VERSION 3.10)
project(cobot_linux)

# Output paths
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Lua51 REQUIRED)

include_directories(${LUA_INCLUDE_DIR})

SET(libs ${LUA_LIBRARIES} )

# Shared headers and source files
set(SHARED_SRCS
  src/shared/math/gvector.cpp
  src/shared/util/helpers.cpp
  src/shared/util/proghelp.cc
  src/shared/util/pthread_utils.cpp
  src/shared/util/terminal_utils.cpp
  src/shared/util/configreader.cpp
  src/shared/util/watch_files.cpp
  src/shared/util/timer.cc
)

add_library(shared_libraries ${SHARED_SRCS})
target_include_directories(shared_libraries PUBLIC
  ${PROJECT_SOURCE_DIR}/src/shared/math
  ${PROJECT_SOURCE_DIR}/src/shared/util
)
target_link_libraries(shared_libraries ${libs})
target_compile_options(shared_libraries PRIVATE -march=native)

# Make shared_libraries linkable to others
ament_target_dependencies(shared_libraries
  rclcpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros sensor_msgs
)

# Add hardware modules
add_subdirectory(src/hardware)

ament_package()
