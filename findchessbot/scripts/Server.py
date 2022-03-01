#!/usr/bin/env python3

import os
from unittest import case
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from findchessbot.srv import AddTwoInts, MoveChesspiece, PoseGripper, PoseJoint
from findchessbot.msg import JogJoint, JogLinear
from math import *
import numpy as np
import cv2
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped


class FindChessBot_Server(Node):

    def __init__(self):
        super().__init__('FindChessBot_Server')
        
        self.findchessbot_state_case = 0
        self.Chessmove = self.create_service(MoveChesspiece, '/findchessbot/cmd/move/chesspiece',self.move_chesspiece_callback)
        # self.JogJointmove = self.create_service(JogJoint, '/findchessbot/cmd/move/jogjoint',self.move_jogjoint_callback)
        # self.JogLinearmove = self.create_service(JogLinear, '/findchessbot/cmd/move/joglinear',self.move_joglinear_callback)
        self.PoseGrippermove = self.create_service(PoseGripper, '/findchessbot/cmd/move/posegripper',self.move_posegripper_callback)
        self.PoseJointmove = self.create_service(PoseJoint, '/findchessbot/cmd/move/posejoint',self.move_posejoint_callback)
        self.JogJointmove = self.create_subscription(JogJoint, '/findchessbot/cmd/move/jogjoint', self.jogjoint_callback, 1)
        self.JogLinearmove = self.create_subscription(JogLinear, '/findchessbot/cmd/move/joglinear', self.joglinear_callback, 1)

        # self.timer = self.create_timer(0.05, self.on_timer)
        # self.timer2 = self.create_timer(60, self.on_timer2)

        # self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

    def move_chesspiece_callback(self, request, response):
        response.result = True
        print(request.chesspiece_pick, request.chesspiece_place, request.chesspiece_move)
        return response

    def move_posegripper_callback(self, request, response):
        response.result = True
        print(request.position_x, request.position_y, request.position_z, request.orientation_yaw)
        return response

    def move_posejoint_callback(self, request, response):
        response.result = True
        print(request.orientation_joint1, request.position_joint2, request.orientation_joint3, request.orientation_joint4)
        return response

    def jogjoint_callback(self, msg):
        print(msg.delta_q1, msg.delta_q2, msg.delta_q3, msg.delta_q4)
    
    def joglinear_callback(self, msg):
        print(msg.delta_x, msg.delta_y, msg.delta_z, msg.delta_yaw)
    
    def update_switchcase(self, mode):
        if mode == 0:
            return 0
    def on_timer(self):
        pass

    def on_timer2(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    Server = FindChessBot_Server()
    Server.get_logger().info("Server Started")
    while rclpy.ok():
        rclpy.spin_once(Server)
        # Server.findchessbot_state_case

if __name__ == '__main__':
    main()
