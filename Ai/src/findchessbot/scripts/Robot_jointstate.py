#!/usr/bin/env python3
from math import sin, cos, pi, atan2 ,sqrt
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32,Float32MultiArray
import numpy as np
from python_lib.findchessbot_kinematic_lib import findchessbot

class Quintic_Traj():
    def __init__(self) -> None:
        pass

    def Traj_Gen(   self,
                    T,
                    Start_pos,
                    Final_pos,
                    Start_velocity,
                    Final_velocity,
                    Start_acceleration,
                    Final_acceleration  ):

        C0 = Start_pos
        C1 = Start_velocity
        C2 = Start_acceleration/2.0



        A = Final_pos - (Start_pos + (Start_velocity*T) + (Start_acceleration*T*T/2))
        B = Final_velocity - (Start_velocity + (Start_acceleration*T))
        C = Final_acceleration - Start_acceleration

        T2 = T*T
        T3 = T*T*T
        T4 = T*T*T*T
        T5 = T*T*T*T*T

        C3 = (10.0*A/T3) - (4.0*B/T2) + (C/(2.0*T))
        C4 = (-15.0*A/T4) + (7.0*B/T3) - (C/T2)
        C5 = (6.0*A/T5) - (3.0*B/T4) + (C/(2.0*T3))

        return C0,C1,C2,C3,C4,C5

    def Traj_Eval(self, C0, C1, C2, C3, C4, C5, t):
        t2 = t*t
        t3 = t*t*t
        t4 = t*t*t*t
        t5 = t*t*t*t*t
    
        S = C0 + (C1*t) + (C2*t2) + (C3*t3) + (C4*t4) + (C5*t5)
        V = C1 + (2.0*C2*t) + (3.0*C3*t2) + (4.0*C4*t3) + (5.0*C5*t4)
        A = (2.0*C2) + (6.0*C3*t) + (12.0*C4*t2) + (20.0*C5*t3)  

        return S, V, A

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        self.theta_q1 = []
        self.theta_q3 = []
        self.i = 0
        self.Robot = findchessbot()
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.subscription = self.create_subscription(Float32,'/findchessbot/chessboard_orientation',self.listener_callback,10)
        # self.subscription = self.create_subscription(Float32MultiArray, topic, callback, qos_profile)
        self.subscription  # prevent unused variable warning
        
        degree = pi / 180.0

        self.joint_state = JointState()
        
        self.chessboard_orientation = 0.

        self.rate = 0.02
        self.timer = self.create_timer(self.rate, self.timer_callback)
        
        self.circle_angle = pi*2.
        
        self.q1 = 0.
        self.q2 = 0.
        self.q3 = 0.
        self.q4 = 0.
        # T = 4
        # self.Traj = Quintic_Traj()
        # self.C0, self.C1, self.C2, self.C3, self.C4, self.C5 = self.Traj.Traj_Gen()

    def timer_callback(self):
        # update joint_state
        # q2 = -0.135
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self.joint_state.position = [self.q1, -0.135, self.q3, self.q4]
        
        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
                

    # def listener_callback_joint_state(self, msg):
    #     self.chessboard_orientation = msg.data

    def listener_callback(self, msg):
        self.chessboard_orientation = msg.data
        q1,q2,q3,q4 = self.Robot.IK(X = (0.247*cos(self.chessboard_orientation+2.356))+0.44, #0.42744
                                    Y = (0.247*sin(self.chessboard_orientation+2.356))-0.00059371, 
                                    Z = 0, 
                                    Yaw = 0 )
        self.q1,self.q2,self.q3,self.q4 = q1,q2,q3,q4

        
# def euler_to_quaternion(roll, pitch, yaw):
#     qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
#     qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
#     qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
#     qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
#     return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    # print("oooo")
    state_publisher = StatePublisher()
    while rclpy.ok():
        rclpy.spin_once(state_publisher)
        # state_publisher.Jacobian()
if __name__ == '__main__':
    main()
