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

# Lidar
```
ros2 run urg_node urg_node_driver --ros-args -p ip_address:=192.168.0.10 -r /scan:=/Cobot/Laser
```

# Temporary Manual Setup
*(We'll automate all this later, but for now, just doing manual setup)*

## AMRL Messages ROS2 Setup
1. Add the path to your `~/.bashrc` file for the `AMENT_PREFIX_PATH` environment variable:
   ```bash
   echo "export AMENT_PREFIX_PATH=$(pwd)/amrl_msgs/install:\$AMENT_PREFIX_PATH" >> ~/.bashrc
   source ~/.bashrc
   ```
2. Run `make` in the amrl_msgs directory to build and install for ROS2.

## AMRL Maps ROS2 Setup
1. Add the path to your `~/.bashrc` file for the `AMENT_PREFIX_PATH` environment variable:
   ```bash
   echo "export AMENT_PREFIX_PATH=$(pwd)/amrl_maps/install:\$AMENT_PREFIX_PATH" >> ~/.bashrc
   source ~/.bashrc
   ```
2. Run `make` in the amrl_maps directory to build and install for ROS2.

## WebViz ROS2 Setup
1. Install system dependencies:
   ```bash
   sudo apt install build-essential cmake qt5-default libqt5websockets5-dev \
                    qtbase5-dev qtwebengine5-dev libgoogle-glog-dev libgflags-dev \
                    colcon-common-extensions libgtest-dev liblua5.1-0-dev
   ```
2. Add the path to your `~/.bashrc` file for the `AMENT_PREFIX_PATH` environment variable:
   ```bash
   echo "export AMENT_PREFIX_PATH=$(pwd)/webviz/install:\$AMENT_PREFIX_PATH" >> ~/.bashrc
   source ~/.bashrc
   ```
3. Run `make` in the webviz directory to build and install for ROS2.

## Additional Bashrc Setup
Add this function to your `~/.bashrc` to automatically source all ROS2 packages in AMENT_PREFIX_PATH:
```bash
# Auto-source all ROS2 packages in AMENT_PREFIX_PATH
function source_ros2_env() {
    if [ -n "$AMENT_PREFIX_PATH" ]; then
        IFS=':' read -ra PATHS <<< "$AMENT_PREFIX_PATH"
        for path in "${PATHS[@]}"; do
            # Skip the base ROS2 path (already sourced)
            if [[ "$path" == "/opt/ros/"* ]]; then
                continue
            fi
            
            # Look for local_setup.bash in share/*/local_setup.bash pattern
            local_setup_files=$(find "$path" -name "local_setup.bash" -path "*/share/*/local_setup.bash" 2>/dev/null)
            
            for setup_file in $local_setup_files; do
                if [ -f "$setup_file" ]; then
                    package_name=$(basename $(dirname "$setup_file"))
                    source "$setup_file"
                fi
            done
        done
    fi
}
source_ros2_env
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