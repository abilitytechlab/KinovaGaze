#!/usr/bin/env python

"""KinovaGaze Node

This file communicates between the KinovaGaze web interface and the Kinova ROS node.

Requires the Kinova ROS node to be running in a separate thread.

Contains a timeout system where a timeout value continuously counts down and stops all arm movement if it reaches 0.
This is a safety feature to stop the arm from moving if the interface disconnects.
The timeout value can be set via a message and should not be set higher than needed. It is automatically set to 1000ms upon receiving any movement command.
Internally, unlock_movement() has to be called before any movement to ensure the arm is unlocked.

Relative and continuous movement is relative to the tool, meaning that movement happens from the perspective of the hand and not the base.
The roll axis of the tool is only applied to translation movements and not rotary movements which are always relative to a plane perpundicular to the base.

It provides the following functions:
    * kinovagaze/tool_move_absolute, type: kinova_msgs.msg.KinovaPose
        Moves the tool to an absolute position in Cartesian space
    * kinovagaze/tool_move_relative, type: kinova_msgs.msg.KinovaPose
        Moves the tool relative to it's current position in Cartesian space
    * kinovagaze/tool_move_continuous, type: kinova_msgs.msg.PoseVelocity
        Moves the tool continuously until a stop message is received, in Cartesian space
    * kinovagaze/finger_position_set, type: kinova_msgs.msg.FingerPosition
        Sets the position of each finger, takes 3 values with with 0 being fully open and 100 being fully closed.
    * kinovagaze/stop_movement, type: std_msgs.msg.Empty
        Stops all movement of the arm
    * kinovagaze/return_home, type: std_msgs.msg.Empty
        Homes the arm, must be called before tool movement commands if the arm has been turned off
    * kinovagaze/msg_send, type: std_msgs.msg.String
        Prints the received message to the console
    * kinovagaze/get_tool_pose, std_msgs.msg.Empty
        Prints the current tool pose to the console
    * kinovagaze/timeout, std_msgs.msg.Int32
        Sets the timeout value, in milliseconds
"""
import sys
import signal
import copy
import rospy
import std_msgs.msg
import kinova_msgs.msg
import kinova_msgs.srv
import geometry_msgs.msg
import actionlib
import math
import time
import numpy as np

from pyquaternion import Quaternion

prefix = "j2n6s300"

class Kinova_Actions:
    """Contains methods for all actions that the node can perform, including the timeout system.

    For the timeout system to work, spin() should be called which uses the current thread for counting down.

    Attributes
    ----------
    enforce_foward : bool
        Whether the tool should be forced into a single direction, effectively disabling rotation
    forward : int[3]
        eulerXYZ degrees direction to set tool if enforce_forward is enabled
    current_tool_pose
        Stores the last received tool pose from the Kinova ROS stack.
    stopped : bool
        True if arm commands are disabled
    timeout : int
        Time until movement is stopped in ms
    prefix : String
        Kinova ROS prefix for arm, by default "j2n6s300"
    max_finger_value : int
        Maximum value for finger range of Kinova arm, by default 6800
    """
    enforce_forward = True
    forward = [90, 0, 0]

    current_tool_pose = None
    stopped = False
    timeout = 1000

    def __init__(self, prefix="j2n6s300", max_finger_value=6800):
        self.prefix = prefix
        self.max_finger_value = max_finger_value

        # self.kinova_continuous_driver = self.Kinova_Continuous_Driver(frequency=100)

        self.finger_position_client = actionlib.SimpleActionClient(
            f'/{self.prefix}_driver/fingers_action/finger_positions',
            kinova_msgs.msg.SetFingersPositionAction)
        self.finger_position_client.wait_for_server()
        rospy.loginfo("Connected to finger_position_server")

        self.tool_pose_client = actionlib.SimpleActionClient(
            f'/{self.prefix}_driver/pose_action/tool_pose',
            kinova_msgs.msg.ArmPoseAction)
        self.tool_pose_client.wait_for_server()
        rospy.loginfo("Connected to tool_pose_server")

        rospy.wait_for_service(f'/{self.prefix}_driver/in/add_pose_to_Cartesian_trajectory')
        self.tool_add_pose_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/add_pose_to_Cartesian_trajectory', kinova_msgs.srv.AddPoseToCartesianTrajectory)
        rospy.loginfo("Connected to add_pose_to_Cartesian_trajectory service")

        rospy.wait_for_service(f'/{self.prefix}_driver/in/clear_trajectories')
        self.tool_clear_trajectories = rospy.ServiceProxy(f'/{self.prefix}_driver/in/clear_trajectories', kinova_msgs.srv.ClearTrajectories)
        rospy.loginfo("Connected to clear_trajectories service")
        
        rospy.wait_for_service(f'/{self.prefix}_driver/in/home_arm')
        self.home_arm_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/home_arm', kinova_msgs.srv.HomeArm)
        rospy.loginfo("Connected to home_arm service")

        rospy.wait_for_service(f'/{self.prefix}_driver/in/stop')
        self.stop_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/stop', kinova_msgs.srv.Stop)
        rospy.loginfo("Connected to stop service")

        rospy.wait_for_service(f'/{self.prefix}_driver/in/start')
        self.start_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/start', kinova_msgs.srv.Start)
        rospy.loginfo("Connected to start service")

        self.movement_publisher = rospy.Publisher(f'/{prefix}_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=10)

    # class Kinova_Continuous_Driver:
    #     """Unused class for sending velocity commands to the arm."""
    #     movement = kinova_msgs.msg.PoseVelocity()
        
    #     def __init__(self, frequency = 100):
    #         self.movement_publisher = rospy.Publisher(f'/{prefix}_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=10)
    #         self.frequency = frequency
    #         self.movement = kinova_msgs.msg.PoseVelocity()
        
    #     def set_movement(self, movement):
    #         rospy.loginfo(movement)
    #         self.movement = movement
        
    #     def stop_movement(self):
    #         self.movement = kinova_msgs.msg.PoseVelocity()

    #     def publish_stop_movement(self):
    #         self.movement_publisher.publish(kinova_msgs.msg.PoseVelocity())

    #     def loop(self):
    #         rospy.loginfo("Starting kinova velocity loop!")
    #         starttime = time.monotonic()
    #         counter = 0
    #         try:
    #             while True:
    #                 time.sleep(0.005)
    #                 # rospy.loginfo((1/self.frequency) - (time.monotonic() - starttime) % (1/self.frequency))
    #                 time.sleep((1/self.frequency) - (time.monotonic() - starttime) % (1/self.frequency))
    #                 if(self.movement != kinova_msgs.msg.PoseVelocity()):
    #                     movement = copy.deepcopy(self.movement)
    #                     self.movement_publisher.publish(movement)
    #                     counter += 1
    #                     if(counter % 1000 == 0):
    #                         rospy.loginfo("Sent 1000 messages, current direction is:")
    #                         rospy.loginfo(self.movement)
    #         except KeyboardInterrupt:
    #             rospy.loginfo("Exiting...")
                
    def spin(self):
        # self.kinova_continuous_driver.loop()
        self.timeout_loop()

    def timeout_loop(self):
        rospy.loginfo("Starting kinova cooldown loop")
        lastUpdate = time.monotonic()
        loop = 0
        try:
            while not exit_program:
                time.sleep(0.005)
                currentTime = time.monotonic()
                if (self.timeout > 0):
                    self.timeout -= (currentTime - lastUpdate)*1000
                    # loop += 1
                    # if(loop >= 1000):
                    #     loop = 0
                    #     print(f'Current timeout: {self.timeout}')
                elif (self.stopped == False):
                    rospy.loginfo("Timeout!")
                    self.stop_movement()
                lastUpdate = currentTime
        except KeyboardInterrupt:
            rospy.loginfo("Exiting...")
    
    def set_timeout_rosmsg(self, milliseconds):
        self.set_timeout(int(milliseconds.data))

    def set_timeout(self, milliseconds, verbose=False):
        if(milliseconds > self.timeout):
            self.timeout=milliseconds
        if(verbose): rospy.loginfo(f"Timeout: {self.timeout}")
    
    def set_finger_position(self, finger_positions):
        rospy.loginfo(f"Setting finger positions to: {finger_positions}")
        self.unlock_movement(verbose=False)
        
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions.finger1)/100*self.max_finger_value
        goal.fingers.finger2 = float(finger_positions.finger2)/100*self.max_finger_value
        goal.fingers.finger3 = float(finger_positions.finger3)/100*self.max_finger_value
        rospy.loginfo("Sending goal...")
        self.finger_position_client.send_goal(goal)
        rospy.loginfo("Waiting for result...")
        if self.finger_position_client.wait_for_result(rospy.Duration(5.0)):
            rospy.loginfo(f"{self.finger_position_client.get_result()}")
            return None
        else:
            self.finger_position_client.cancel_all_goals()
            rospy.loginfo('        the gripper action timed-out')
            return None
        
    def tool_move_continuous(self, movement):
        """Moves tool continuously in a direction relative to current tool pose until stopped.

        Uses the Kinova ROS add pose service to construct a path which continuously moves in a direction relative to the starting tool pose.
        Movement is not truely endless as it will stop after about 1 meter of translation movement or about 9 full rotations.
        
        Arguments
        ---------
        movement : kinova_msgs.msg.PoseVelocity
            Axis to move in, each axis should be 1, 0, or -1
        """
        self.stop_movement(verbose=False)
        self.unlock_movement(verbose=False)

        starting_pose = tool_pose_to_EulerXYZ(self.current_tool_pose)

        local_movement = rotate_movement_towards_tool(self.current_tool_pose, 
                                                      [movement.twist_linear_x, 
                                                       movement.twist_linear_y, 
                                                       movement.twist_linear_z, 
                                                       movement.twist_angular_x, 
                                                       movement.twist_angular_y, 
                                                       movement.twist_angular_z], 
                                                       verbose=False)
        
        movement = [movement.twist_linear_x, 
                    movement.twist_linear_y, 
                    movement.twist_linear_z, 
                    movement.twist_angular_x, 
                    movement.twist_angular_y, 
                    movement.twist_angular_z]

        movement_direction = [
            local_movement[0]*.1,
            local_movement[1]*.1,
            local_movement[2]*.1,
            local_movement[3]*1.5,
            local_movement[4]*1.5,
            local_movement[5]*1.5
        ]

        forward = Degrees2Radians(self.forward)

        rospy.loginfo("Starting continuous motion")
        rospy.loginfo(f"Starting pose: \n{starting_pose}")
        rospy.loginfo(f"Movement direction: \n{movement}")
        rospy.loginfo(f"Local movement direction: \n{movement_direction}")
        try:
            for i in range(10):
                response = None
                if(self.enforce_forward):
                    response = self.tool_add_pose_service(
                        starting_pose[0] + i*movement_direction[0],
                        starting_pose[1] + i*movement_direction[1],
                        starting_pose[2] + i*movement_direction[2],
                        forward[0],
                        forward[1],
                        forward[2]
                    )
                else:
                    response = self.tool_add_pose_service(
                        starting_pose[0] + i*movement_direction[0],
                        starting_pose[1] + i*movement_direction[1],
                        starting_pose[2] + i*movement_direction[2],
                        starting_pose[3] + i*movement_direction[3],
                        starting_pose[4] + i*movement_direction[4],
                        starting_pose[5] + i*movement_direction[5]
                    )
                # rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.loginfo("add_pose_to_Cartesian_trajectory or  service call failed: %s"%e)

    def tool_move_relative(self, pose):
        """Moves tool relative to the current tool pose in meters and radians.

        Arguments
        ---------
        pose : kinova_msgs.msg.KinovaPose
            Relative target position in meters and radians.
        """
        self.stop_movement(verbose=False)
        self.unlock_movement(verbose=False)

        starting_pose = tool_pose_to_EulerXYZ(self.current_tool_pose)

        relative_pose = rotate_movement_towards_tool(self.current_tool_pose, [pose.X, pose.Y, pose.Z, pose.ThetaX, pose.ThetaY, pose.ThetaZ], verbose=False)

        rospy.loginfo("Starting continuous motion")
        rospy.loginfo(f"Starting pose: \n{starting_pose}")
        rospy.loginfo(f"Relative target pose: \n{pose}")
        rospy.loginfo(f"Local relative target pose: \n{relative_pose}")
        
        pose.X = starting_pose[0] + relative_pose[0]
        pose.Y = starting_pose[1] + relative_pose[1]
        pose.Z = starting_pose[2] + relative_pose[2]
        if(self.enforce_forward):
            forward = Degrees2Radians(self.forward)
            pose.ThetaX = forward[0]
            pose.ThetaY = forward[1]
            pose.ThetaZ = forward[2]
        else:
            pose.ThetaX = starting_pose[3] + relative_pose[3]
            pose.ThetaY = starting_pose[4] + relative_pose[4]
            pose.ThetaZ = starting_pose[5] + relative_pose[5]

        try:
            response = self.tool_add_pose_service(
                # pose[0],
                # pose[1],
                # pose[2],
                # pose[3],
                # pose[4],
                # pose[5]
                pose.X,
                pose.Y,
                pose.Z,
                pose.ThetaX,
                pose.ThetaY,
                pose.ThetaZ
            )
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.loginfo("add_pose_to_Cartesian_trajectory or  service call failed: %s"%e)

    def tool_move_absolute(self, pose):
        """Moves tool in absolute space with the origin approxiately at the robot base, in meters and radians.

        Arguments
        ---------
        pose : kinova_msgs.msg.KinovaPose
            Target position in meters and radians.
        """
        self.stop_movement(verbose=False)
        self.unlock_movement(verbose=False)

        goal = kinova_msgs.msg.ArmPoseGoal()
        
        goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + '_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x = pose.X,
            y = pose.Y,
            z = pose.Z
        )

        orientation = None
        if(self.enforce_forward):
            orientation = EulerXYZ2Quaternion(Degrees2Radians([self.forward[0], self.forward[1], self.forward[2]]))
        else:
            orientation = EulerXYZ2Quaternion(Degrees2Radians([pose.ThetaX, pose.ThetaY, pose.ThetaZ]))
        
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x = orientation[0],
            y = orientation[1],
            z = orientation[2],
            w = orientation[3]
        )

        rospy.loginfo(goal)

        # rospy.loginfo("Sending goal...")
        self.tool_pose_client.send_goal(goal)
        # rospy.loginfo("Waiting for result...")
        if self.tool_pose_client.wait_for_result(rospy.Duration(10.0)):
            rospy.loginfo(self.tool_pose_client.get_state())
        else:
            rospy.loginfo('The cartesian action timed-out.')
            self.tool_pose_client.cancel_all_goals()
            return None

    def return_home(self, argument = None):
        """Homes the arm, calibrating it in the process.
        """
        self.unlock_movement(verbose=False)
        rospy.loginfo("Homing arm...")
        try:
            response = self.home_arm_service()
            rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.loginfo("home_arm service call failed: %s"%e)

    def stop_movement(self, verbose=True):
        """Stops and disables all movement of the arm.

        Arguments
        ---------
        verbose : bool, optional
            Set to False to only print error output, by default True
        """

        if(verbose):
            rospy.loginfo(f'Stopping all movements.')

        try:
            response_stop = self.stop_service()
            response_clear_trajectories = self.tool_clear_trajectories()
            self.stopped = True
            if(verbose == True): 
                rospy.loginfo(response_stop, response_clear_trajectories)
        except rospy.ServiceException as e:
            rospy.loginfo("stop service call failed: %s"%e)

    def unlock_movement(self, verbose=True):
        """Unlocks movement of the arm after it has been disabled,

        Arguments
        ---------
        verbose : bool, optional
            Set to False to only print error output, by default True
        """
        try:
            response = self.start_service()
            if(verbose == True): 
                rospy.loginfo(response)
        except rospy.ServiceException as e:
            rospy.loginfo("stop service call failed: %s"%e)

        self.set_timeout(1000)
        
        self.stopped = False
        
    def get_tool_pose(self, argument = None):
        """Print last received tool pose.
        """
        rospy.loginfo(self.current_tool_pose)

    def message_from_GUI(self, data):
        """Prints content of message data

        Arguments
        ---------
        data : std_msgs.msg.String
            Message with string to print
        """
        rospy.loginfo(f'Message from GUI: {data}')
        rospy.loginfo(rospy.get_caller_id() + 'Message from GUI: %s', data.data)

    def receive_tool_pose(self, toolpose):
        """Callback for tool pose message from Kinova ROS, stores pose as class attribute.

        Argument
        --------
        toolpose : geometry_msgs.msg.PoseStamped
            Current pose of tool
        """
        self.current_tool_pose = toolpose

def Degrees2Radians(degrees, verbose=False):
    """Translates a list of values in degrees to a list of values in radians
    
    Arguments
    ---------
    degrees : list of float
        List of angles in degrees to be translated
    verbose : bool, optional
        Set to True to enable log output, by default False

    Returns
    -------
    list of float
        List of angles in radians
    """
    if (verbose): rospy.loginfo(f"Degrees: {degrees}")
    constant = 2*math.pi/360.0
    radians = []
    for value in degrees:
        radians.append(value*constant)
    if (verbose): rospy.loginfo(f"Converted to Radians: {radians}")
    return radians

def AddEulerToQuaternion(EulerXYZ_, Q_input):
    """Adds EulerXYZ radian in list form to quaternion in list form
    
    Arguments
    ---------
    EulerXYZ_ : list of float
        List of 3 floats representing an euler 3D angle in radians
    Q_input : list of float
        List of 4 floats representing a quaternion

    Returns
    -------
    list of float
        List of 4 floats representing a quaternion, sum of EulerXYZ_ and Q_input
    """
    EulerInput = Quaternion2EulerXYZ(Q_input)
    sumEulerXYZ_ = [
        EulerXYZ_[0] + EulerInput[0],
        EulerXYZ_[1] + EulerInput[1],
        EulerXYZ_[2] + EulerInput[2]
    ]
    sumQuaternion = EulerXYZ2Quaternion(sumEulerXYZ_)
    return sumQuaternion

def QuaternionNorm(Q_raw):
    """Normalises a quaternion in list form.
    
    Arguments
    ---------
    Q_raw : list of float
        List of 4 floats representing a quaternion

    Returns
    -------
    list of float
        List of 4 floats representing a quaternion
    """
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_

def Quaternion2EulerXYZ(Q_raw, verbose=False):
    """Converts a quaternion in list form to EulerXYZ radians in list form.
    
    Arguments
    ---------
    Q_raw: list of float
        List of 4 floats representing a quaternion
    verbose : bool, optional
        Set to True to enable log output, by default False

    Returns
    -------
    list of float
        List of 3 floats representing an euler 3D angle in radians
    """
    if (verbose): rospy.loginfo(f"Quaternion: {Q_raw}")
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    if (verbose): rospy.loginfo(f"Converted to EulerXYZ: {EulerXYZ_}")
    return EulerXYZ_

def EulerXYZ2Quaternion(EulerXYZ_, verbose=False):
    """Converts EulerXYZ radians in list form to a quaternion in list form.

    Parameters
    ----------
    EulerXYZ_ : list of float
        List of 3 floats representing an euler 3D angle in radians
    verbose : bool, optional
        Set to True to enable log output, by default False

    Returns
    -------
    list of float
        List of 4 floats representing a quaternion
    """
    if (verbose): rospy.loginfo(f"EulerXYZ: {EulerXYZ_}")
    tx_, ty_, tz_ = EulerXYZ_[0:3]
    sx = math.sin(0.5 * tx_)
    cx = math.cos(0.5 * tx_)
    sy = math.sin(0.5 * ty_)
    cy = math.cos(0.5 * ty_)
    sz = math.sin(0.5 * tz_)
    cz = math.cos(0.5 * tz_)

    qx_ = sx * cy * cz + cx * sy * sz
    qy_ = -sx * cy * sz + cx * sy * cz
    qz_ = sx * sy * cz + cx * cy * sz
    qw_ = -sx * sy * sz + cx * cy * cz

    Q_ = [qx_, qy_, qz_, qw_]
    if (verbose): rospy.loginfo(f"Converted to Quaternion: {Q_}")
    return Q_

def tool_pose_to_EulerXYZ(pose):
    """Converts Kinova tool pose to vector with EulerXYZ in radians

    Parameters
    ----------
    pose : geometry_msgs.msg.PoseStamped
        Kinova tool pose

    Returns
    -------
    list of float
        List of 6 floats representing a vector and EulerXYZ radians
    """
    rotation = Quaternion2EulerXYZ([
        pose.pose.orientation.x, 
        pose.pose.orientation.y, 
        pose.pose.orientation.z, 
        pose.pose.orientation.w
    ])

    return [
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z,
        rotation[0],
        rotation[1],
        rotation[2]
    ]

def rotate_movement_towards_tool(tool_pose, movement, verbose=False):
    """Rotates a givem movement to become relative to the current tool pose

    Parameters
    ----------
    tool_pose : geometry_msgs.msg.PoseStamped
        Kinova tool pose
    movement : list of float
        List of 6 floats representing a vector and EulerXYZ radians
    verbose : bool, optional
        Set to True to enable log output, by default False

    Returns
    -------
    list of float
        List of 6 floats representing a vector and EulerXYZ radians
    """
    if (verbose):
        rospy.loginfo("Tool orientation:")
        rospy.loginfo(Quaternion2EulerXYZ([
            tool_pose.pose.orientation.x,
            tool_pose.pose.orientation.y, 
            tool_pose.pose.orientation.z, 
            tool_pose.pose.orientation.w]))
        rospy.loginfo("Movement before:")
        rospy.loginfo(movement)

    tool_rotation = Quaternion(
        tool_pose.pose.orientation.w,
        tool_pose.pose.orientation.x,
        tool_pose.pose.orientation.y, 
        tool_pose.pose.orientation.z
        )
    
    if (verbose): rospy.loginfo(f"tool_rotation_axis: {tool_rotation.axis}")
    # tool_rotation_offset = Quaternion(axis=[1, 0, 0], angle=math.pi / 2)
    # tool_rotation = tool_rotation + tool_rotation_offset

    linear_movement = [
        movement[0],
        movement[2],
        -movement[1]
    ]
    rotated_linear_movement = tool_rotation.rotate(linear_movement)

    # tool_rotation_z = Quaternion(
    #     axis=[0, 0, 1.0], radians = Quaternion2EulerXYZ([
    #         tool_pose.pose.orientation.w,
    #         tool_pose.pose.orientation.x, 
    #         tool_pose.pose.orientation.y, 
    #         tool_pose.pose.orientation.z
    #     ])[2]
    #     )
    
    # angular_movement = [
    #     -movement[3],
    #     movement[4],
    #     movement[5]
    # ]
    # rotated_angular_movement = tool_rotation_z.rotate(angular_movement)
    
    rotated_movement = copy.copy(movement)
    rotated_movement[0] = rotated_linear_movement[0]
    rotated_movement[1] = rotated_linear_movement[1]
    rotated_movement[2] = rotated_linear_movement[2]
    # rotated_movement[3] = rotated_angular_movement[0]
    # rotated_movement[4] = rotated_angular_movement[1]
    # rotated_movement[5] = rotated_angular_movement[2]

    if(verbose):
        rospy.loginfo("Movement after:")
        rospy.loginfo(rotated_movement)
    
    return rotated_movement

exit_program = False
def signal_handler(signal, frame):
    """Handle signals to stop program gracefully.
    """
    rospy.loginfo("\nprogram exiting gracefully")
    global exit_program 
    exit_program = True
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def setup_subscribers(kinova_actions):
    """Subscribes to messages from GUI and Kinova ROS stack

    Parameters
    ----------
    kinova_actions : Instance of Kinova_Actions
        Instance of Kinova_Actions with callback methods
    """
    rospy.Subscriber('kinovagaze/tool_move_absolute', kinova_msgs.msg.KinovaPose, kinova_actions.tool_move_absolute)
    rospy.Subscriber('kinovagaze/tool_move_relative', kinova_msgs.msg.KinovaPose, kinova_actions.tool_move_relative)
    rospy.Subscriber('kinovagaze/tool_move_continuous', kinova_msgs.msg.PoseVelocity, kinova_actions.tool_move_continuous)
    rospy.Subscriber('kinovagaze/finger_position_set', kinova_msgs.msg.FingerPosition, kinova_actions.set_finger_position)
    rospy.Subscriber('kinovagaze/stop_movement', std_msgs.msg.Empty, kinova_actions.stop_movement)
    rospy.Subscriber('kinovagaze/return_home', std_msgs.msg.Empty, kinova_actions.return_home)
    rospy.Subscriber('kinovagaze/msg_send', std_msgs.msg.String, kinova_actions.message_from_GUI)
    rospy.Subscriber('kinovagaze/get_tool_pose', std_msgs.msg.Empty, kinova_actions.get_tool_pose)
    rospy.Subscriber('kinovagaze/timeout', std_msgs.msg.Int32, kinova_actions.set_timeout_rosmsg)
    
    rospy.Subscriber(f'/{prefix}_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, kinova_actions.receive_tool_pose)

    rospy.loginfo("Spun up GUI and Kinova-ROS listeners")

if __name__ == '__main__':
    print("Starting...")
    rospy.loginfo("Starting KinovaGaze Node...")
    rospy.init_node('listener')
    kinova_actions = Kinova_Actions(prefix="j2n6s300", max_finger_value=6800)
    setup_subscribers(kinova_actions)
    rospy.loginfo("Ready!")
    # rospy.spin()
    kinova_actions.spin()
