import argparse
import time
import threading
import numpy as np

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2, Base_pb2


def parseConnectionArguments(parser = argparse.ArgumentParser()):
    parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
    parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
    return parser.parse_args()


class DeviceConnection:
    
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args): 
        """
        returns RouterClient required to create services and send requests to device or sub-devices,
        """

        return DeviceConnection(args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password))

    @staticmethod
    def createUdpConnection(args): 
        """        
        returns RouterClient that allows to create services and send requests to a device or its sub-devices @ 1khz.
        """

        return DeviceConnection(args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password))

    def __init__(self, ipAddress, port=TCP_PORT, credentials = ("","")):

        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    # Called when entering 'with' statement
    def __enter__(self):
        
        self.transport.connect(self.ipAddress, self.port)

        if (self.credentials[0] != ""):
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000   # (milliseconds)
            session_info.connection_inactivity_timeout = 2000 # (milliseconds)

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    # Called when exiting 'with' statement
    def __exit__(self, exc_type, exc_value, traceback):
    
        if self.sessionManager != None:

            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000 
            
            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()


# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check

def check_for_end(e):
    """Return a closure checking for END notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END:
            e.set()
    return check


def send_cartesian_twist(base, delta, debug=False):
    
    command = Base_pb2.TwistCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    command.duration = 0

    twist = command.twist
    twist.linear_x = delta[1]
    twist.linear_y = delta[2]
    twist.linear_z = delta[0]
    twist.angular_x = delta[3]
    twist.angular_y = delta[4]
    twist.angular_z = delta[5]

    base.SendTwistCommand(command)

    # Let time for twist to be executed
    time.sleep(0.1)

    base.Stop()

def send_cartesian_wrench(base, delta, duration, debug=False):
    
    if duration < 1:
        print("Warning: the duration is set as %f" % duration)

    command = Base_pb2.WrenchCommand()

    command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_TOOL
    # command.duration = duration # not sure how to use this

    wrench = command.wrench
    wrench.force_x = delta[1]   # limit: 40.0 N
    wrench.force_y = delta[2]
    wrench.force_z = delta[0]
    wrench.torque_x = delta[4]  # limit: 15.0 N·m
    wrench.torque_y = delta[5]
    wrench.torque_z = delta[3]

    base.SendWrenchCommand(command)

    # Let time for wrench to be executed
    time.sleep(duration)
    base.Stop()

def send_cartesian_action(base, base_cyclic, delta, TIMEOUT_DURATION, debug=False):
    if debug:
        print("Starting Cartesian action movement ...")
    
    action = Base_pb2.Action()
    action.name = "Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = feedback.base.tool_pose_x + delta[0]             # (meters)
    cartesian_pose.y = feedback.base.tool_pose_y + delta[1]             # (meters)
    cartesian_pose.z = feedback.base.tool_pose_z + delta[2]             # (meters)
    cartesian_pose.theta_x = feedback.base.tool_pose_theta_x + delta[3] # (degrees)
    cartesian_pose.theta_y = feedback.base.tool_pose_theta_y + delta[4] # (degrees)
    cartesian_pose.theta_z = feedback.base.tool_pose_theta_z + delta[5] # (degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end(e),
        Base_pb2.NotificationOptions()
    )

    if debug:
        print("Executing action")
    base.ExecuteAction(action)

    if debug:
        print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION) # Maximum allowed waiting time during actions (in seconds)
    base.Unsubscribe(notification_handle)

    if debug:
        if finished:
            print("Cartesian movement completed")
        else:
            print("Timeout on action notification wait")
    return finished


def send_gripper_action(base, position):
    np.clip(position, 0., 1.)

    # Create the GripperCommand we will send later
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # set gripper position
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    finger.finger_identifier = 1
    finger.value = position
    base.SendGripperCommand(gripper_command)
    time.sleep(0.5)


def move_to_position(base, position_name="Home", TIMEOUT_DURATION=30):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == position_name:
            action_handle = action.handle
    # import pdb; pdb.set_trace()

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished