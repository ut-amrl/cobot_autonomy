import sys
import time
import signal
from enum import Enum, auto
import numpy as np

# kinova arm imports
from utils.arm_utils import parseConnectionArguments, DeviceConnection
from utils.arm_utils import send_gripper_action, send_cartesion_twist
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

# ros imports
import rclpy
from rclpy.node import Node
from cobot_msgs.msg import CobotJoystickMsg

# custom imports
from spacemouse import SpaceMouse

GRIPPER_DIM = -1
ARM_SCALE_TRANSLATION = 3.0  # current maximum: 3.0
ARM_SCALE_ROTATION = 30.0 / 0.375  # current maximum: 30.0
BASE_SCALE_TRANSLATION = 0.4  # current maximum: 0.4
BASE_SCALE_ROTATION = 0.6 / 0.375  # current maximum: 0.6

def handle_sigint(sig, frame):
    print("Shutting down gracefully...")
    # any custom cleanup here
    sys.exit(0)
    
def check_valid_action(action, use_gripper=False):
    # check translation/rotation
    if np.any(action[:GRIPPER_DIM]):
        return True
    # check gripper
    if use_gripper:
        if np.any(action[GRIPPER_DIM] > 0.0):
            return True
    return False

def filter_arm_action(action, threshold_factor=0.5):
    maximum_translation = ARM_SCALE_TRANSLATION
    maximum_rotation = ARM_SCALE_ROTATION * 0.375
    threshold_translation = threshold_factor * maximum_translation
    threshold_rotation = threshold_factor * maximum_rotation
    action[0:3][np.abs(action[0:3]) <= threshold_translation] = 0.0
    action[3:GRIPPER_DIM][np.abs(action[3:GRIPPER_DIM]) <= threshold_rotation] = 0.0
    return action

def filter_base_action(action, threshold_factor=0.1):
    maximum_translation = BASE_SCALE_TRANSLATION
    maximum_rotation = BASE_SCALE_ROTATION * 0.375
    threshold_translation = threshold_factor * maximum_translation
    threshold_rotation = threshold_factor * maximum_rotation
    action[0:3][np.abs(action[0:3]) <= threshold_translation] = 0.0
    action[3:GRIPPER_DIM][np.abs(action[3:GRIPPER_DIM]) <= threshold_rotation] = 0.0
    return action

def input2action(device, controller_type="OSC_POSE", robot_name="Panda", gripper_dof=1):
    state = device.get_controller_state()
    # Note: Devices output rotation with x and z flipped to account for robots starting with gripper facing down
    #       Also note that the outputted rotation is an absolute rotation, while outputted dpos is delta pos
    #       Raw delta rotations from neutral user input is captured in raw_drotation (roll, pitch, yaw)
    dpos, rotation, raw_drotation, grasp, switch_mode = (
        state["dpos"],
        state["rotation"],
        state["raw_drotation"],
        state["grasp"],
        state["switch_mode"],
    )

    dpos = -dpos
    drotation = raw_drotation #[[1, 0, 2]]

    action = None

    assert controller_type == "OSC_POSE"
    drotation[2] = -drotation[2]
    drotation *= 75
    dpos *= 200
    drotation = drotation

    grasp = 1 if grasp else -1
    action = np.concatenate([dpos, drotation, [grasp] * gripper_dof])

    return action, grasp, switch_mode

class Mode(Enum):
    STOPPED = auto()
    MANUAL_BASE = auto()
    MANUAL_ARM = auto()
    AUTONOMOUS = auto()

    # repeat [1, 2..., end -> start]
    def next(self):
        n = len(list(self.__class__))
        return Mode(self.value + 1 if self.value < n - 1 else 0)

class SpaceMouseInterface(Node):
    def __init__(self, 
                 space_mouse: SpaceMouse, 
                 arm_router: DeviceConnection,
                 gripper_opened: bool = True, # TODO: this should be read from gripper state
    ):
        super().__init__('spacemouse_driver')
        self.space_mouse = space_mouse
        
        # Set up arm connection
        # Note (Taijing): This connection skips ros
        self.arm_base_client = BaseClient(arm_router)
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.arm_base_client.SetServoingMode(base_servo_mode)
        
        # TODO: this should be read from gripper state
        self.gripper_opened = gripper_opened
        self.mode = Mode.STOPPED
        
        self._setup_publisher()
        
    def _setup_publisher(self):
        self.joystick_pub = self.create_publisher(
            CobotJoystickMsg,
            '/Cobot/Joystick/Dummy',  # topic name
            10  # QoS queue depth
        )
        
    def action_dofs(self, dofs):
        # stop mode
        if self.mode == Mode.STOPPED:  
            pass

        # navigation mode (base)
        elif self.mode == Mode.MANUAL_BASE:  
            dofs[0:3] *= BASE_SCALE_TRANSLATION
            dofs[3:GRIPPER_DIM] *= BASE_SCALE_ROTATION
            dofs = filter_base_action(dofs)
            if check_valid_action(dofs, use_gripper=False):
                x = dofs[0]
                y = dofs[1]
                r = -dofs[5]
                
                # self.cobot.apply_base_drive(x=x, y=y, r=r)
                NumAxes = 5 # Note (Taijing): I don't understand why this is 5, but it works with the current joystick message
                joystickMsg = CobotJoystickMsg()
                joystickMsg.buttons = 0x0
                joystickMsg.axes = [0.0 for _ in range(NumAxes)]
                joystickMsg.axes[0] = x
                joystickMsg.axes[1] = y
                joystickMsg.axes[2] = r
                self.joystick_pub.publish(joystickMsg)

        # manipulation mode (arm)
        elif self.mode == Mode.MANUAL_ARM: 
            dofs[0:3] *= ARM_SCALE_TRANSLATION
            dofs[3:GRIPPER_DIM] *= ARM_SCALE_ROTATION
            dofs[3] = -dofs[3]
            dofs = filter_arm_action(dofs)

            if check_valid_action(dofs, use_gripper=True):
                if dofs[GRIPPER_DIM] == 1.0:  # button clicked
                    # self.cobot.apply_gripper_action(1.0 if self.gripper_opened else 0.0)
                    send_gripper_action(self.arm_base_client, 1.0 if self.gripper_opened else 0.0)
                    self.gripper_opened = not self.gripper_opened
                else:
                    # self.cobot.apply_arm_action(dofs, type="twist")
                    send_cartesion_twist(self.arm_base_client, dofs)
                    
        elif self.mode == Mode.AUTONOMOUS: 
            pass # autonomous mode not implemented yet
        
        else:
            print("Unknown mode:", self.mode)
            return
        
    def action_right_click(self):
        self.mode = self.mode.next()
        print("Spacemouse mode:", self.mode)
        time.sleep(0.5)
        
    def action_left_click(self):
        pass
        
    def run(self):
        while True:
            controller_type = "OSC_POSE"
            dofs, left_click, right_click = input2action(
                device=self.space_mouse,
                controller_type=controller_type,
            )

            if right_click:
                self.action_right_click()
            elif left_click:
                self.action_left_click()
            else:
                # for now we handle the left click behavior in dofs
                self.action_dofs(dofs)

if __name__ == "__main__":
    # Handle graceful shutdown
    signal.signal(signal.SIGINT, handle_sigint)
    
    # Set up space mouse
    import hid
    hid_devices = hid.enumerate()
    spacemouse = None
    for device in hid_devices:
        if 'SpaceMouse' in device.get('product_string'):
            spacemouse = device
            break
    if spacemouse:
        vendor_id = spacemouse['vendor_id']
        product_id = spacemouse['product_id']
        device_path = spacemouse['path']
        print(f"Vendor ID: {vendor_id}, Product ID: {product_id}, Device Path: {device_path}")
    else:
        print("SpaceMouse device not found.")
        exit(1)
    space_mouse = SpaceMouse(vendor_id=vendor_id, product_id=product_id)
    space_mouse.start_control()
    
    # Set up arm connection
    # Note (Taijing): This connection skip ros
    args = parseConnectionArguments()
    
    rclpy.init()
    with DeviceConnection.createTcpConnection(args) as arm_router:
        driver = SpaceMouseInterface(spacemouse, arm_router)
        driver.run()
    rclpy.shutdown()