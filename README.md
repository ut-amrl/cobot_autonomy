# cobot_autonomy
UT AMRL stack for Cobot MoMa

# Container Setup
See [CONTAINER.md](CONTAINER.md)

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

# Temporary Manual Setup
*(We'll automate all this later, but for now, just doing manual setup)*

1. `cd cobot_autonomy`
2. Add following to `~/.bashrc`
```bash
export AMENT_PREFIX_PATH=$(pwd)/amrl_msgs/install:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$(pwd)/amrl_maps/install:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$(pwd)/webviz/install:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$(pwd)/enml/install:$AMENT_PREFIX_PATH
export AMENT_PREFIX_PATH=$(pwd)/graph_navigation/install:$AMENT_PREFIX_PATH

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

cmdrepeat() {
  local n=$1; shift
  for ((i=0; i<n; i++)); do
    "$@"
  done
}
export -f cmdrepeat

```
3. Build all submodules:
```
cd cobot_autonomy
cd amrl_maps && cmdrepeat 4 make -j4 && cd ..
cd amrl_msgs && cmdrepeat 4 make -j4 && cd ..
source ~/.bashrc
cd webviz && cmdrepeat 4 make -j4 && cd ..
cd enml && cmdrepeat 4 make -j4 && cd ..
cd graph_navigation && cmdrepeat 4 make -j4 && cd ..
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

# Launch WebViz
1. Run the websocket server:
```bash
cd webviz && ./bin/websocket --config_file=../config/cobot_webviz_config.lua --v=1
```
2. Open `webviz/webviz.html` in a web browser
3. Enter robot IP and click Connect

# Launch Enml
```
cd enml && ./bin/enml -c ../config -r enml.lua
```

# Launch Graph Nav
```
cd graph_navigation && ./bin/navigation -robot_config ../config/navigation.lua
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