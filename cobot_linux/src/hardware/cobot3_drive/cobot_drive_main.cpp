#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <sys/time.h>

#include "drive.h"              
#include "terminal_utils.h"     
#include "proghelp.h"           
#include "configreader.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "cobot_msgs/msg/cobot_drive_msg.hpp"
#include "cobot_msgs/msg/cobot_odometry_msg.hpp"

int main(int argc, char **argv) {
    std::cout << "Cobot Drive Main Program" << std::endl;
}