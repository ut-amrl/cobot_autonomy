import subprocess
import signal
import os
import sys
import time
import rclpy
from rclpy.logging import get_logger

# Store all subprocesses
processes = []
logger = get_logger("cobot_startup")

def launch_background(cmd, cwd=None):
    logger.info(f"[launch] (cwd={cwd or os.getcwd()}) {' '.join(cmd)}")
    p = subprocess.Popen(
        cmd,
        preexec_fn=os.setsid,
        cwd=cwd
    )
    processes.append(p)

def cleanup():
    logger.info("Shutting down...")
    for p in processes:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass

def wait_for_node(node_name, timeout=10.0, interval=0.2):
    logger.info(f"[wait_for_node] Waiting for node '{node_name}'...")
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            output = subprocess.check_output(["ros2", "node", "list"]).decode()
            if node_name in output.splitlines():
                logger.info(f"[wait] Node '{node_name}' is up.")
                return True
        except subprocess.CalledProcessError:
            pass
        time.sleep(interval)
    logger.warn(f"[wait] Timeout waiting for node '{node_name}'")
    return False

def launch_urg_node():
    launch_background([
        "ros2", "run", "urg_node", "urg_node_driver",
        "--ros-args",
        "-p", "ip_address:=192.168.0.10",
        "-r", "/scan:=/Cobot/Laser"
    ])
    wait_for_node("/urg_node")
    launch_background(["ros2", "param", "set", "/urg_node", "angle_min", "-1.0"])
    launch_background(["ros2", "param", "set", "/urg_node", "angle_max", "1.0"])

def launch_cobot3_drive():
    launch_background(["./bin/cobot3_drive"], cwd="cobot_linux")

def launch_spacemouse_driver():
    launch_background(
        ["python3", "spacemouse/spacemouse_driver.py"]
    )

def main():
    def parse_args():
        import argparse
        parser = argparse.ArgumentParser()
        parser.add_argument("--enable-teleop", action="store_true")
        args = parser.parse_args()
        return args
    args = parse_args()
    
    rclpy.init()
    try:
        launch_urg_node()
        launch_cobot3_drive()
        if args.enable_teleop:
            launch_spacemouse_driver()
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        cleanup()
        sys.exit(0)

if __name__ == "__main__":
    main()
