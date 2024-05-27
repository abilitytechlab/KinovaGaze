#!/usr/bin/env python

import copy
import rospy
import std_msgs.msg
import kinova_msgs.msg
import kinova_msgs.srv
import geometry_msgs.msg
import actionlib
import math
import time

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
        
        rospy.wait_for_service(f'/{self.prefix}_driver/in/home_arm')
        self.home_arm_service = rospy.ServiceProxy(f'/{self.prefix}_driver/in/home_arm', kinova_msgs.srv.HomeArm)
        print("Connected to home_arm_service")

    class Kinova_Continuous_Driver:
        movement = kinova_msgs.msg.PoseVelocity()
        
        def __init__(self, frequency = 100):
            self.movement_publisher = rospy.Publisher(f'/{prefix}_driver/in/cartesian_velocity', kinova_msgs.msg.PoseVelocity, queue_size=10)
            self.frequency = frequency
        
        def set_movement(self, movement):
            print(movement)
            self.movement = movement
        
        def stop_movement(self):
            self.movement = kinova_msgs.msg.PoseVelocity()

        def loop(self):
            print("Starting kinova velocity loop!")
            starttime = time.monotonic()
            while True:
                movement = copy.deepcopy(self.movement)
                self.movement_publisher.publish(movement)
                time.sleep(0.005)
                time.sleep(self.frequency - ((time.monotonic() - starttime) % self.frequency))

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
        self.kinova_continuous_driver.set_movement(movement)
        
    def tool_move_continuous(self, axis):
        print(f"Starting continuous movement in axis: {axis}")

        goal = kinova_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + '_link_base'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
        x=self.current_tool_pose.pose.position.x+axis.twist_linear_x*.2, 
        y=self.current_tool_pose.pose.position.y+axis.twist_linear_y*.2,
        z=self.current_tool_pose.pose.position.z+axis.twist_linear_z*.2
        )

        goal_rotation = AddEulerToQuaternion(
            [
                axis.twist_angular_x*1.5, 
                axis.twist_angular_y*1.5, 
                axis.twist_angular_z*1.5
                ],
            [
                self.current_tool_pose.pose.orientation.x, 
                self.current_tool_pose.pose.orientation.y, 
                self.current_tool_pose.pose.orientation.z, 
                self.current_tool_pose.pose.orientation.w
                ]   
            )

        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=goal_rotation[0],
        y=goal_rotation[1],
        z=goal_rotation[2],
        w=goal_rotation[3]
        )

        print(goal)

        print("Sending goal...")
        self.tool_pose_client.send_goal(goal)
        print("Waiting for result...")
        if self.tool_pose_client.wait_for_result(rospy.Duration(10.0)):
            print("Succeeded!")
        else:
            print('The cartesian action timed-out.')
            self.tool_pose_client.cancel_all_goals()
            return None

    def return_home(self, argument):
        print("Homing arm...")
        try:
            response = self.home_arm_service()
            print(response)
        except rospy.ServiceException as e:
            print("home_arm service call failed: %s"%e)

    def stop_movement(self, argument):
        print('Stopping all movements')
        # self.kinova_continuous_driver.stop_movement()
        self.tool_pose_client.cancel_all_goals()
        self.finger_position_client.cancel_all_goals()
        

    def get_tool_pose(self, argument):
        print(self.current_tool_pose)

    def message_from_GUI(self, data):
        print(f'Message from GUI: {data}')
        rospy.loginfo(rospy.get_caller_id() + 'Message from GUI: %s', data.data)

    def receive_tool_pose(self, toolpose):
        self.current_tool_pose = toolpose

def setup_subscribers(kinova_actions):
    rospy.Subscriber('kinovagaze/finger_position_set', kinova_msgs.msg.FingerPosition, kinova_actions.set_finger_position)
    rospy.Subscriber('kinovagaze/tool_move_continuous', kinova_msgs.msg.PoseVelocity, kinova_actions.tool_move_continuous)
    rospy.Subscriber('kinovagaze/stop_movement', std_msgs.msg.Empty, kinova_actions.stop_movement)
    rospy.Subscriber('kinovagaze/return_home', std_msgs.msg.Empty, kinova_actions.return_home)
    rospy.Subscriber('kinovagaze/msg_send', std_msgs.msg.String, kinova_actions.message_from_GUI)
    rospy.Subscriber('kinovagaze/get_tool_pose', std_msgs.msg.Empty, kinova_actions.get_tool_pose)

    rospy.Subscriber(f'/{prefix}_driver/out/tool_pose', geometry_msgs.msg.PoseStamped, kinova_actions.receive_tool_pose)

    print("Spun up GUI and Kinova-ROS listeners")


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

def Quaternion2EulerXYZ(Q_raw):
    Q_normed = QuaternionNorm(Q_raw)
    qx_ = Q_normed[0]
    qy_ = Q_normed[1]
    qz_ = Q_normed[2]
    qw_ = Q_normed[3]

    tx_ = math.atan2((2 * qw_ * qx_ - 2 * qy_ * qz_), (qw_ * qw_ - qx_ * qx_ - qy_ * qy_ + qz_ * qz_))
    ty_ = math.asin(2 * qw_ * qy_ + 2 * qx_ * qz_)
    tz_ = math.atan2((2 * qw_ * qz_ - 2 * qx_ * qy_), (qw_ * qw_ + qx_ * qx_ - qy_ * qy_ - qz_ * qz_))
    EulerXYZ_ = [tx_,ty_,tz_]
    return EulerXYZ_

def EulerXYZ2Quaternion(EulerXYZ_):
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
    return Q_

if __name__ == '__main__':
    print("Starting...")
    rospy.init_node('listener')
    kinova_actions = Kinova_Actions(prefix="j2n6s300", max_finger_value=6800)
    setup_subscribers(kinova_actions)
    print("Ready!")
    rospy.spin()
    # kinova_actions.spin()
