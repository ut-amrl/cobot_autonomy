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

To compile linux cobot:
```
cd cobot_linux
make 
cd ..
```

# Teleop
In one terminal, run:
```
cd cobot_linux && ./bin/cobot3_drive
```
In another terminal:
```
python spacemouse/spacemouse_driver.py
```

# Known Issues
If cobot fails to connect to lidar:
```
sudo lsof -i :10940
```
You are expected to see the following:
```
COMMAND    PID    USER   FD   TYPE DEVICE SIZE/OFF NODE NAME
urg_node 83308 tiejean   23u  IPv4 415360      0t0  TCP cobot:48444->hokuyo:10940 (ESTABLISHED)
```
Kill this process by running:
```
sudo kill -9 <PID>
```
and try again.