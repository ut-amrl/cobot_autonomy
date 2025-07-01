SHELL = /bin/bash

# Detect ROS1 (rosbuild) based on ROS_ROOT env var
CHECK_ROS1 = $(shell if [ -n "$$ROS_ROOT" ] && [ -f "$$ROS_ROOT/core/rosbuild/rosbuild.cmake" ]; then echo 1; else echo 0; fi)

# Build type for CMake
build_type = Release

.PHONY: all install clean build

all:
ifeq ($(CHECK_ROS1),1)
	$(info Detected ROS1 (rosbuild). Building with CMake ...)
	$(MAKE) build build/CMakeLists.txt.copy
	$(MAKE) --no-print-directory -C build
else
	$(info Detected ROS2 (ament). Building with colcon ...)
	colcon build --packages-select cobot_msgs --cmake-args -DCMAKE_BUILD_TYPE=$(build_type)
endif

install:
ifeq ($(CHECK_ROS1),1)
	@echo "ROS1 detected. 'make install' not needed. Use ROS_PACKAGE_PATH."
else
	@echo "Installing ROS2 (ament) package to ./install ..."
	colcon build --packages-select cobot_msgs --install-base install --cmake-args -DCMAKE_BUILD_TYPE=$(build_type)
endif

clean:
	rm -rf build install log msg_gen lib

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile msg
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build
