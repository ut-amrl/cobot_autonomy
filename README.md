# cobot_autonomy
UT AMRL stack for Cobot MoMa

# Installation
To compile the messages:
```
cd cobot_msgs
make && make install
```

Add the following to `~/.bashrc`:
```
source /opt/ros/foxy/setup.bash
source ~/cobot_autonomy/cobot_msgs/install/setup.sh --extend
export ROS_PACKAGE_PATH=~/cobot_autonomy/cobot_msgs:$ROS_PACKAGE_PATH
```