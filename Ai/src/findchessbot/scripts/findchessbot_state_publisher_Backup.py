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


class findchessbot():
    def __init__(self):
        self.L1 = 0.013245
        self.L2 = 0.370
        self.L3 = 0.315
        self.L12 = self.L1 + self.L2
        self.h1 = 0.125 
        self.h3 = 0.065
        self.h4 = 0.190

        self.C1 = 1
        self.C13 = 1
        self.C134 = 1
        self.C3 = 1
        self.S1 = 0
        self.S13 = 0
        self.S134 = 0
        self.S3 = 1

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0

        self.pose_X = 0.745
        self.pose_Y = 0
        self.pose_Z = 0
        self.pose_Yaw = 0

        self.H1, self.H2, self.H3, self.H4, self.He = self.FK(q1=0, q2=0, q3=0, q4=0, init=1)

        self.Je, self.Je_star, self.Je_inv = self.Jacobian(init = 1)
        self.Coorchess_to_point = {  
                'A1' : [0.247, 2.356],  'A2' : [0.215, 2.191],  'A3' : [0.19, 1.976],   'A4' : [0.177, 1.713],
                'A5' : [0.177, 1.429],  'A6' : [0.19, 1.166],   'A7' : [0.215, 0.951],  'A8' : [0.247, 0.785],
                'B1' : [0.215, 2.521],  'B2' : [0.177, 2.356],  'B3' : [0.146, 2.111],  'B4' : [0.127, 1.768],
                'B5' : [0.127, 1.373],  'B6' : [0.146, 1.03],   'B7' : [0.177, 0.785],  'B8' : [0.215, 0.62],
                'C1' : [0.19, 2.737],   'C2' : [0.146, 2.601],  'C3' : [0.106, 2.356],  'C4' : [0.079, 1.893],
                'C5' : [0.079, 1.249],  'C6' : [0.106, 0.785],  'C7' : [0.146, 0.54],   'C8' : [0.19, 0.405],
                'D1' : [0.177, 3.0],    'D2' : [0.127, 2.944],  'D3' : [0.079, 2.82],   'D4' : [0.035, 2.356],
                'D5' : [0.035, 0.785],  'D6' : [0.079, 0.322],  'D7' : [0.127, 0.197],  'D8' : [0.177, 0.142],
                'E1' : [0.177, 3.283],  'E2' : [0.127, 3.339],  'E3' : [0.079, 3.463],  'E4' : [0.035, 3.927],
                'E5' : [0.035, 5.498],  'E6' : [0.079, 5.961],  'E7' : [0.127, 6.086],  'E8' : [0.177, 6.141],
                'F1' : [0.19, 3.546],   'F2' : [0.146, 3.682],  'F3' : [0.106, 3.927],  'F4' : [0.079, 4.391],
                'F5' : [0.079, 5.034],  'F6' : [0.106, 5.498],  'F7' : [0.146, 5.743],  'F8' : [0.19, 5.878],
                'G1' : [0.215, 3.762],  'G2' : [0.177, 3.927],  'G3' : [0.146, 4.172],  'G4' : [0.127, 4.515],
                'G5' : [0.127, 4.91],   'G6' : [0.146, 5.253],  'G7' : [0.177, 5.498],  'G8' : [0.215, 5.663],
                'H1' : [0.247, 3.927],  'H2' : [0.215, 4.092],  'H3' : [0.19, 4.307],   'H4' : [0.177, 4.57],
                'H5' : [0.177, 4.854],  'H6' : [0.19, 5.117],   'H7' : [0.215, 5.333],  'H8' : [0.247, 5.498], }

    def IK(self,X,Y,Z,Yaw):
        C3 = ((X*X) + (Y*Y) - (pow(self.L12,2)) - (pow(self.L3,2)) ) / (2*self.L12*self.L3)
        S3 = sqrt(1-pow(C3,2))
        q3 = atan2(S3,C3)
        L3s3 = self.L3*S3
        L123c3 = self.L12+(self.L3*C3)
        S1 = (-L3s3*X) + (L123c3*Y)
        C1 = (L3s3*Y) + (L123c3*X)
        q1 = atan2(S1,C1)
        q4 = Yaw - q1 - q3
        q2 = Z+self.h4-self.h3-self.h1

        self.FK(q1=q1, q2=q2, q3=q3, q4=q4, init=1)

        return q1,q2,q3,q4

    def FK(self,q1,q2,q3,q4,init = 0):
        C1 = cos(q1)
        S1 = sin(q1)
        C13 = cos(q1+q3)
        S13 = sin(q1+q3)
        C134 = cos(q1 + q3 + q4)
        S134 = sin(q1 + q3 + q4)

        H1 =   [[C1,    -S1,    0,  0],
                [S1,    C1,     0,  0],
                [0,     0,      1,  self.h1],
                [0,     0,      0,  1]]

        H2 =   [[C1,    -S1,    0,  self.L1*C1],
                [S1,    C1,     0,  self.L1*S1],
                [0,     0,      1,  self.h1+q2],
                [0,     0,      0,  1]]
        
        H3 =   [[C13,    -S13,      0,      (self.L1+self.L2)*C1],
                [S13,    C13,       0,      (self.L1+self.L2)*S1],
                [0,     0,          1,       self.h1+self.h3+q2],
                [0,     0,          0,      1]]

        H4 =   [[C134,    -S134,    0,      (self.L3*C13)+((self.L1+self.L2)*C1)],
                [S134,    C134,     0,      (self.L3*S13)+((self.L1+self.L2)*S1)],
                [0,     0,      1,          self.h1+self.h3+q2],
                [0,     0,      0,          1]]
        
        He =   [[C134,    -S134,    0,      (self.L3*C13)+((self.L1+self.L2)*C1)],
                [S134,    C134,     0,      (self.L3*S13)+((self.L1+self.L2)*S1)],
                [0,     0,      1,          self.h1+self.h3-self.h4+q2],
                [0,     0,      0,          1]]

        
        if init == 0:
            self.H1 = H1
            self.H2 = H2
            self.H2 = H3
            self.H4 = H4
            self.He = He

            self.q1 = q1
            self.q2 = q2
            self.q3 = q3
            self.q4 = q4

            self.pose_X = He[0][3]
            self.pose_Y = He[1][3]
            self.pose_Z = He[2][3]
            self.pose_Yaw = q1 + q3 + q4

            self.C1 = C1
            self.C13 = C13
            self.C134 = C134
            self.C3 = cos(q3)
            self.S1 = S1
            self.S13 = S13
            self.S134 = S134
            self.S3 = sin(q3)

        self.Jacobian(init = 0)
        return H1, H2, H3, H4, He
        
    def Jacobian(self,init = 0):
        Je = [  [                                           0,    0,                0,      0],
                [                                           0,    0,                0,      0],
                [                                           1,    0,                1,      1],
                [(-self.L3*self.S13) - (self.L1*self.S1) - (self.L2*self.S1),    0,    (-self.L3*self.S13),     0],
                [ (self.L3*self.C13) + (self.L1*self.C1) + (self.L2*self.C1),    0,    (self.L3*self.C13),      0],
                [                                           0,    1,                0,      0]]
        
        # if int(self.q3*1000) not in range(-210,210):        
        #     Je_inv = [  [0,                                    self.C13/(self.S3*(self.L12)),                                    self.S13/(self.S3*self.L12), 0],
        #                 [0,                                                                   0,                                                                   0, 1],
        #                 [0, -((self.L3*self.C13) + (self.L1*self.C1) + (self.L2*self.C1)) / (self.L3*self.S3*self.L12), -((self.L3*self.S13) + (self.L1*self.S1) + (self.L2*self.S1))/(self.L3*self.S3*self.L12), 0],
        #                 [1,                                                self.C1/(self.L3*self.S3),                                                self.S1/(self.L3*self.S3), 0]]
        
        # else:
        #     Je_inv = None

        Je_inv = [  [0,                                    self.C13/(self.S3*(self.L12)),                                    self.S13/(self.S3*self.L12), 0],
                        [0,                                                                   0,                                                                   0, 1],
                        [0, -((self.L3*self.C13) + (self.L1*self.C1) + (self.L2*self.C1)) / (self.L3*self.S3*self.L12), -((self.L3*self.S13) + (self.L1*self.S1) + (self.L2*self.S1))/(self.L3*self.S3*self.L12), 0],
                        [1,                                                self.C1/(self.L3*self.S3),                                                self.S1/(self.L3*self.S3), 0]]
        if init == 0:
            self.Je = Je
            self.Je_star = Je[2:5]
            self.Je_inv = Je_inv
        return Je,Je[2:5],Je_inv

    def IVK(self):
        return 0
    
    def IAK(self):
        return 0

    def Circle_Traj(self, Coor, Chessboard_Theta=0):
        r,theta = self.Coorchess_to_point[Coor]
        # X = r*cos(theta) + 0.745
        # Y = r*sin(theta) 
        theta_now = (theta + Chessboard_Theta)%(pi * 2)
        list_of_theta = np.arange(theta_now, pi*2, 0.02).tolist() + np.arange(0, theta_now, 0.02).tolist()
        # list_of_theta = [round(num, 3) for num in list_of_pose]

        list_of_pose = [(round((r*cos(theta_loop))+0.425, 3),round(r*sin(theta_loop), 3)) for theta_loop in list_of_theta]
        return list_of_pose
        
    def Cubic_Tarj(self, Initial, Goal):
        Coff1 = 0
        Coff2 = 0
        Coff3 = 0
        Coff4 = 0
        return Coff1,Coff2,Coff3,Coff4

    def Quintic_Tarj(self):
        
        return Coff1,Coff2,Coff3,Coff4,Coff5,Coff6



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

        self.subscription = self.create_subscription(Float32,'/findchessbot/chessboard_rpm',self.listener_callback,10)
        # self.subscription = self.create_subscription(Float32MultiArray, topic, callback, qos_profile)
        self.subscription  # prevent unused variable warning
        
        degree = pi / 180.0

        self.joint_state = JointState()
        
        self.chessboard_orientation = 0.

        self.rate = 0.01
        self.timer = self.create_timer(self.rate, self.timer_callback)
        
        self.circle_angle = pi*2.
        
        self.q1 = 0.
        self.q2 = 0.
        self.q3 = 0.
        self.q4 = 0.

    def timer_callback(self):
        # update joint_state
        # q2 = -0.135
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_chess']
        self.joint_state.position = [self.q1, -0.135, self.q3, self.q4, self.chessboard_orientation]
        
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

        
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    # print("oooo")
    state_publisher = StatePublisher()
    while rclpy.ok():
        rclpy.spin_once(state_publisher)
        # state_publisher.Jacobian()
if __name__ == '__main__':
    main()
