#!/usr/bin/env python
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
    current_tool_pose = None

    def __init__(self, prefix="j2n6s300", max_finger_value=6800):
        self.prefix = prefix
        self.max_finger_value = max_finger_value

        self.kinova_continuous_driver = self.Kinova_Continuous_Driver(frequency=100)

        self.finger_position_client = actionlib.SimpleActionClient(
            f'/{self.prefix}_driver/fingers_action/finger_positions',
            kinova_msgs.msg.SetFingersPositionAction)
        self.finger_position_client.wait_for_server()
        print("Connected to finger_position_server")

        self.tool_pose_client = actionlib.SimpleActionClient(
            f'/{self.prefix}_driver/pose_action/tool_pose',
            kinova_msgs.msg.ArmPoseAction)
        self.tool_pose_client.wait_for_server()
        print("Connected to tool_pose_server")

        rospy.wait_for_service(f'/{self.prefix}_driver/in/add_pose_to_Cartesian_trajectory')
        self.tool_add_pose_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/add_pose_to_Cartesian_trajectory', kinova_msgs.srv.AddPoseToCartesianTrajectory)
        print("Connected to add_pose_to_Cartesian_trajectory")

        rospy.wait_for_service(f'/{self.prefix}_driver/in/clear_trajectories')
        self.tool_clear_trajectories = rospy.ServiceProxy(f'/{self.prefix}_driver/in/clear_trajectories', kinova_msgs.srv.ClearTrajectories)
        print("Connected to clear_trajectories")
        
        rospy.wait_for_service(f'/{self.prefix}_driver/in/home_arm')
        self.home_arm_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/home_arm', kinova_msgs.srv.HomeArm)
        print("Connected to home_arm_service")

    class Kinova_Continuous_Driver:
        movement = kinova_msgs.msg.PoseVelocity()
        
        def __init__(self, frequency = 100):
            self.movement_publisher = rospy.Publisher(f'/{prefix}_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=10)
            self.frequency = frequency
            self.movement = kinova_msgs.msg.PoseVelocity()
        
        def set_movement(self, movement):
            print(movement)
            self.movement = movement
        
        def stop_movement(self):
            self.movement = kinova_msgs.msg.PoseVelocity()

        def publish_stop_movement(self):
            self.movement_publisher.publish(kinova_msgs.msg.PoseVelocity())

        def loop(self):
            print("Starting kinova velocity loop!")
            starttime = time.monotonic()
            counter = 0
            try:
                while True:
                    time.sleep(0.005)
                    # print((1/self.frequency) - (time.monotonic() - starttime) % (1/self.frequency))
                    time.sleep((1/self.frequency) - (time.monotonic() - starttime) % (1/self.frequency))
                    if(self.movement != kinova_msgs.msg.PoseVelocity()):
                        movement = copy.deepcopy(self.movement)
                        self.movement_publisher.publish(movement)
                        counter += 1
                        if(counter % 1000 == 0):
                            print("Sent 1000 messages, current direction is:")
                            print(self.movement)
            except KeyboardInterrupt:
                print("Exiting...")
                

    def spin(self):
        self.kinova_continuous_driver.loop()
    
    def set_finger_position(self, finger_positions):
        print(f"Setting finger positions to: {finger_positions}")
        
        goal = kinova_msgs.msg.SetFingersPositionGoal()
        goal.fingers.finger1 = float(finger_positions.finger1)/100*self.max_finger_value
        goal.fingers.finger2 = float(finger_positions.finger2)/100*self.max_finger_value
        goal.fingers.finger3 = float(finger_positions.finger3)/100*self.max_finger_value
        print("Sending goal...")
        self.finger_position_client.send_goal(goal)
        print("Waiting for result...")
        if self.finger_position_client.wait_for_result(rospy.Duration(5.0)):
            print(f"{self.finger_position_client.get_result()}")
            return None
        else:
            self.finger_position_client.cancel_all_goals()
            print('        the gripper action timed-out')
            return None
    
    def tool_move_continuous_old(self, movement):
        
        # movement = rotate_movement_towards_tool(self.current_tool_pose, movement)

        self.kinova_continuous_driver.set_movement(movement)
        
    def tool_move_continuous(self, movement):
        self.stop_movement(quick=True)

        starting_pose = tool_pose_to_EulerXYZ(self.current_tool_pose)

        local_movement = rotate_movement_towards_tool(self.current_tool_pose, movement, verbose=True)

        movement_direction = [
            local_movement.twist_linear_x*.05,
            local_movement.twist_linear_y*.05,
            local_movement.twist_linear_z*.05,
            local_movement.twist_angular_x*1.5,
            local_movement.twist_angular_y*1.5,
            local_movement.twist_angular_z*1.5
        ]

        print("Starting continuous motion")
        print(f"Starting pose: \n{starting_pose}")
        print(f"Movement direction: \n{movement}")
        print(f"Local movement direction: \n{movement_direction}")
        try:
            for i in range(5):
                response = self.tool_add_pose_service(
                    starting_pose[0] + i*movement_direction[0],
                    starting_pose[1] + i*movement_direction[1],
                    starting_pose[2] + i*movement_direction[2],
                    starting_pose[3] + i*movement_direction[3],
                    starting_pose[4] + i*movement_direction[4],
                    starting_pose[5] + i*movement_direction[5]
                )
            print(response)
        except rospy.ServiceException as e:
            print("add_pose_to_Cartesian_trajectory or  service call failed: %s"%e)

    def move_to_position(self, pose):
        goal = kinova_msgs.msg.ArmPoseGoal()

        self.stop_movement(quick=True)
        
        goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + '_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            x = pose.X,
            y = pose.Y,
            z = pose.Z
        )
        orientation = EulerXYZ2Quaternion(Degrees2Radians([pose.ThetaX, pose.ThetaY, pose.ThetaZ]))
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            x = orientation[0],
            y = orientation[1],
            z = orientation[2],
            w = orientation[3]
        )

        print(goal)

        print("Sending goal...")
        self.tool_pose_client.send_goal(goal)
        print("Waiting for result...")
        if self.tool_pose_client.wait_for_result(rospy.Duration(10.0)):
            print(self.tool_pose_client.get_state())
        else:
            print('The cartesian action timed-out.')
            self.tool_pose_client.cancel_all_goals()
            return None

    def return_home(self, argument = None):
        print("Homing arm...")
        try:
            response = self.home_arm_service()
            print(response)
        except rospy.ServiceException as e:
            print("home_arm service call failed: %s"%e)

    def stop_movement(self, quick=False):
        loops = 3
        if(quick != True): 
            quick = False
            loops = 5
        print(f'Stopping all movements, quick = {quick}')
        for i in range(loops):
            self.kinova_continuous_driver.stop_movement()
            self.tool_pose_client.cancel_all_goals()
            self.finger_position_client.cancel_all_goals()
            try:
                response = self.tool_clear_trajectories()
                print(response)
            except rospy.ServiceException as e:
                print("tool_clear_trajectories service call failed: %s"%e)
            self.kinova_continuous_driver.publish_stop_movement()
            if(quick == False):
                print(i)
                time.sleep(0.05*i)

        
        
    def get_tool_pose(self, argument = None):
        print(self.current_tool_pose)

    def message_from_GUI(self, data):
        print(f'Message from GUI: {data}')
        rospy.loginfo(rospy.get_caller_id() + 'Message from GUI: %s', data.data)

    def receive_tool_pose(self, toolpose):
        self.current_tool_pose = toolpose

def Degrees2Radians(degrees, verbose=False):
    if (verbose): print(f"Degrees: {degrees}")
    constant = 2*math.pi/360.0
    radians = []
    for value in degrees:
        radians.append(value*constant)
    if (verbose): print(f"Converted to Radians: {radians}")
    return radians

def AddEulerToQuaternion(EulerXYZ_, Q_input):
    EulerInput = Quaternion2EulerXYZ(Q_input)
    sumEulerXYZ_ = [
        EulerXYZ_[0] + EulerInput[0],
        EulerXYZ_[1] + EulerInput[1],
        EulerXYZ_[2] + EulerInput[2]
    ]
    sumQuaternion = EulerXYZ2Quaternion(sumEulerXYZ_)
    return sumQuaternion

def QuaternionNorm(Q_raw):
    qx_temp,qy_temp,qz_temp,qw_temp = Q_raw[0:4]
    qnorm = math.sqrt(qx_temp*qx_temp + qy_temp*qy_temp + qz_temp*qz_temp + qw_temp*qw_temp)
    qx_ = qx_temp/qnorm
    qy_ = qy_temp/qnorm
    qz_ = qz_temp/qnorm
    qw_ = qw_temp/qnorm
    Q_normed_ = [qx_, qy_, qz_, qw_]
    return Q_normed_

def Quaternion2EulerXYZ(Q_raw, verbose=False):
    if (verbose): print(f"Quaternion: {Q_raw}")
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    print(f"Converted to EulerXYZ: {EulerXYZ_}")
    return EulerXYZ_

def EulerXYZ2Quaternion(EulerXYZ_, verbose=False):
    if (verbose): print(f"EulerXYZ: {EulerXYZ_}")
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
    if (verbose): print(f"Converted to Quaternion: {Q_}")
    return Q_

def tool_pose_to_EulerXYZ(pose):
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
        if (verbose):
            print("Tool orientation:")
            print(Quaternion2EulerXYZ([
                tool_pose.pose.orientation.x,
                tool_pose.pose.orientation.y, 
                tool_pose.pose.orientation.z, 
                tool_pose.pose.orientation.w]))
            print("Movement before:")
            print(movement)

        tool_rotation = Quaternion(
            tool_pose.pose.orientation.w,
            tool_pose.pose.orientation.x,
            tool_pose.pose.orientation.y, 
            tool_pose.pose.orientation.z
            )
        if (verbose): print(f"tool_rotation_axis: {tool_rotation.axis}")
        # tool_rotation_offset = Quaternion(axis=[1, 0, 0], angle=math.pi / 2)
        # tool_rotation = tool_rotation + tool_rotation_offset

        linear_movement = [
            movement.twist_linear_x,
            movement.twist_linear_z,
            -movement.twist_linear_y
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
        #     -movement.twist_angular_y,
        #     movement.twist_angular_x,
        #     movement.twist_angular_z,
        # ]
        # rotated_angular_movement = tool_rotation_z.rotate(angular_movement)
        
        rotated_movement = copy.copy(movement)
        rotated_movement.twist_linear_x = rotated_linear_movement[0]
        rotated_movement.twist_linear_y = rotated_linear_movement[1]
        rotated_movement.twist_linear_z = rotated_linear_movement[2]
        # rotated_movement.twist_angular_x = rotated_angular_movement[0]
        # rotated_movement.twist_angular_y = rotated_angular_movement[1]
        # rotated_movement.twist_angular_z = rotated_angular_movement[2]

        if(verbose):
            print("Movement after:")
            print(rotated_movement)
        
        return rotated_movement

def correct_path(start, direction, pose):
    pass

def signal_handler(signal, frame):
    print("\nprogram exiting gracefully")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def setup_subscribers(kinova_actions):
    rospy.Subscriber('kinovagaze/finger_position_set', kinova_msgs.msg.FingerPosition, kinova_actions.set_finger_position)
    rospy.Subscriber('kinovagaze/tool_move_continuous', kinova_msgs.msg.PoseVelocity, kinova_actions.tool_move_continuous)
    rospy.Subscriber('kinovagaze/stop_movement', std_msgs.msg.Empty, kinova_actions.stop_movement)
    rospy.Subscriber('kinovagaze/return_home', std_msgs.msg.Empty, kinova_actions.return_home)
    rospy.Subscriber('kinovagaze/msg_send', std_msgs.msg.String, kinova_actions.message_from_GUI)
    rospy.Subscriber('kinovagaze/get_tool_pose', std_msgs.msg.Empty, kinova_actions.get_tool_pose)
    rospy.Subscriber('kinovagaze/tool_move_position', kinova_msgs.msg.KinovaPose, kinova_actions.move_to_position)

    rospy.Subscriber(f'/{prefix}_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, kinova_actions.receive_tool_pose)

    print("Spun up GUI and Kinova-ROS listeners")

if __name__ == '__main__':
    print("Starting...")
    rospy.init_node('listener')
    kinova_actions = Kinova_Actions(prefix="j2n6s300", max_finger_value=6800)
    setup_subscribers(kinova_actions)
    print("Ready!")
    # rospy.spin()
    kinova_actions.spin()
