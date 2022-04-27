#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from findchessbot.srv import Capture, Castling, PicknPlace, PromotePawn, McuMode
from findchessbot.msg import JogJoint, JogLinear
from math import *
import numpy as np
from python_lib.CRC16 import *
import serial
import time

class MCU_Server(Node):

    def __init__(self):
        super().__init__('MCU_Server')
        self.chess_squares = {  'a1':  0,  'a2':  1, 'a3':  2, 'a4':  3, 'a5':  4, 'a6':  5, 'a7':  6, 'a8':  7, 
                                'b1':  8,  'b2':  9, 'b3': 10, 'b4': 11, 'b5': 12, 'b6': 13, 'b7': 14, 'b8': 15, 
                                'c1': 16,  'c2': 17, 'c3': 18, 'c4': 19, 'c5': 20, 'c6': 21, 'c7': 22, 'c8': 23, 
                                'd1': 24,  'd2': 25, 'd3': 26, 'd4': 27, 'd5': 28, 'd6': 29, 'd7': 30, 'd8': 31, 
                                'e1': 32,  'e2': 33, 'e3': 34, 'e4': 35, 'e5': 36, 'e6': 37, 'e7': 38, 'e8': 39, 
                                'f1': 40,  'f2': 41, 'f3': 42, 'f4': 43, 'f5': 44, 'f6': 45, 'f7': 46, 'f8': 47, 
                                'g1': 48,  'g2': 49, 'g3': 50, 'g4': 51, 'g5': 52, 'g6': 53, 'g7': 54, 'g8': 55, 
                                'h1': 56,  'h2': 57, 'h3': 58, 'h4': 59, 'h5': 60, 'h6': 61, 'h7': 62, 'h8': 63,
                                '99': 99, '00': 123}

        # Mode0 = Ready State, Initial State
        # Mode1 = Control State
        # Mode2 = Jog Joint State
        # Mode3 = Linear Joint State

        self.Server_MCU_Mode = 0
        self.MCU_Mode = 0
        self.Update_Mode_State = 0
        self.MCU_mode_service = self.create_service(McuMode, '/findchessbot/mcu_mode', self.mcu_mode_srv_callback)
        self.test = 0
        # self.MCU_mode_service = self.create_service(McuMode, '/findchessbot/mcu_mode', self.mcu_mode_srv_callback)
        self.Capture = self.create_service(Capture, '/findchessbot/capture',self.capture_callback)
        self.Castling = self.create_service(Castling, '/findchessbot/castling',self.castling_callback)
        self.PicknPlace = self.create_service(PicknPlace, '/findchessbot/picknplace',self.picknplace_callback)
        self.PromotePawn = self.create_service(PromotePawn, '/findchessbot/promotepawn',self.promotepawn_callback)
        self.JogJointmove = self.create_subscription(JogJoint, '/findchessbot/jogjoint', self.jogjoint_callback, 1)
        self.JogLinearmove = self.create_subscription(JogLinear, '/findchessbot/joglinear', self.joglinear_callback, 1)
        self.crc16 = CRC16()
    
    def mcu_mode_srv_callback(self, request, response):
        New_Mode = request.update_mode
        if (self.Server_MCU_Mode != 0):
            # Check State Mbed
            if (New_Mode == 0):
                self.Server_MCU_Mode = 0
                self.MCU_Mode = 0
                if(self.MCU_Mode != 0):
                    response.result = False
                else:
                    response.result = True
        else:
            response.result = True
        if (response.result):
            self.Server_MCU_Mode = New_Mode
            self.MCU_Mode = New_Mode
            self.get_logger().info("Chage Mode to " + str(self.MCU_Mode) +" Done!!")
        else:
            self.get_logger().info("Failed to Chage Mode")
        # print(request.position_x, request.position_y, request.position_z, request.orientation_yaw)
        
        return response

    def Mode1(self, Command, State_Gripper): # Mode1 = Control State
        if self.MCU_Mode == 1:
            Command = Command.lower()
            frame = [0x86,self.chess_squares[Command],State_Gripper]
            data = frame + self.crc16.calculate(frame)
            self.get_logger().info(str(Command) + " : " + str(data))
            try:
                self.H7.write(data)
                time.sleep(0.6)
                self.get_logger().info("Send Command Successful")
                # if self.H7.read(1) == 0xFF:
                #     self.get_logger().info("Send Command Successful")
                # else:
                #     self.get_logger().error("MCU Port Not Avaiable")
                    
            except:
                self.get_logger().error("Can't Communication to MCU")
        else:
            self.get_logger().warning("Please Change the Mode Before Using Chesssquare Mode")

    def Mode2(self, Command): # Mode2 = Jog Joint State
        if self.MCU_Mode == 2:
            frame = [0x61,  Command.angular_velocity_q1, Command.linear_velocity_q2, 
                            Command.angular_velocity_q3, Command.angular_velocity_q4]
            data = frame + self.crc16.calculate(frame)
            self.get_logger().info("Joint Jog\n"    + "Wq1: " + str(Command.angular_velocity_q1)
                                                    + "Vq2: " + str(Command.linear_velocity_q2)
                                                    + "Wq3: " + str(Command.angular_velocity_q3)
                                                    + "Wq4: " + str(Command.angular_velocity_q4))
            try:
                self.H7.write(data)
                time.sleep(0.6)
                self.get_logger().info("Send Command Successful")
                # if self.H7.read(1) == 0xFF:
                #     self.get_logger().info("Send Command Successful")
                # else:
                #     self.get_logger().error("MCU Port Not Avaiable")
            except:
                self.get_logger().error("Can't Communication to MCU")
        else:
            self.get_logger().warning("Please Change the Mode Before Using Joint Jog Mode")
    def Mode3(self, Command): # Mode3 = Linear Joint State
        if self.MCU_Mode == 3:
            frame = [0x71,  Command.linear_velocity_x, Command.linear_velocity_y, 
                            Command.linear_velocity_z, Command.angular_velocity_yaw]
            data = frame + self.crc16.calculate(frame)
            self.get_logger().info("Linear Jog\n"   + "Vx: " + str(Command.linear_velocity_x)
                                                    + "Vy: " + str(Command.linear_velocity_y)
                                                    + "Vz: " + str(Command.linear_velocity_z)
                                                    + "Wyaw: " + str(Command.angular_velocity_yaw))
            try:
                self.H7.write(data)
                time.sleep(0.6)
                self.get_logger().info("Send Command Successful")
                # if self.H7.read(1) == 0xFF:
                #     self.get_logger().info("Send Command Successful")
                # else:
                #     self.get_logger().error("MCU Port Not Avaiable")
            except:
                self.get_logger().error("Can't communication to MCU")
        else:
            self.get_logger().warning("Please Change the Mode Before Using Linear Jog Mode")

    def on_timer(self):
        # self.test += 1
        print("qqqqq")
        pass

    def jogjoint_callback(self, msg):
        if self.Server_MCU_Mode == 2:
            self.get_logger().info('Jog Joint Callback')
            self.Mode2(msg)
        else:
            self.get_logger().warning("Please change robot mode before using Joint-Jog mode")

    
    def joglinear_callback(self, msg):
        if self.Server_MCU_Mode == 3:
            self.get_logger().info('Linear Joint Callback')
            self.Mode3(msg)
        else:
            self.get_logger().warning("Please change robot mode before using Linear-Jog mode")

        

    def capture_callback(self, request, response):
        response.result = True
        self.get_logger().info("Capture Callback")
        self.Mode1(request.chess_square_eat,1)
        self.Mode1('99',2)
        self.Mode1(request.chess_square_pick,1)
        self.Mode1(request.chess_square_place,2)
        self.Mode1('99',2)
        self.enable_control()
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def castling_callback(self, request, response):
        response.result = True
        print("Castling Callback")
        self.Mode1(request.chess_square_pick_king,1)
        self.Mode1(request.chess_square_place_king,2)
        self.Mode1(request.chess_square_pick_rook,1)
        self.Mode1(request.chess_square_place_rook,2)
        self.Mode1('99',2)
        self.enable_control()
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def picknplace_callback(self, request, response):
        response.result = True
        print("Picknplace Callback")
        # print(type(request.chess_square_pick))
        self.Mode1(request.chess_square_pick,1)
        self.Mode1(request.chess_square_place,2)
        self.Mode1('99',2)
        self.enable_control()

        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def promotepawn_callback(self, request, response):
        response.result = True
        print("Promotepawn Callback")
        self.Mode1(request.chess_square_pick_pawn,1)
        self.Mode1(request.chess_square_pawn_to_trash,2)
        self.Mode1(request.chess_square_pick_queen,1)
        self.Mode1(request.chess_square_place,2)
        self.Mode1('99',2)
        self.enable_control()
        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def enable_control(self):
        control_state = 41
        frame1 = [0x87,control_state]
        data = frame1 + self.crc16.calculate(frame1)
        self.H7.write(data)
        
def main(args=None):
    rclpy.init(args=args)
    Server = MCU_Server()
    Server.get_logger().info("Serial Comunication OK!!")
    
    try:
        Server.H7 = serial.Serial('/dev/ttyACM0', 115200, timeout=5)
        # print("")
    except:
        Server.get_logger().error("Serial Communication not available")
        Check = False;
    else:
        Server.get_logger().info("Serial Communication enable")
        Check = True;

    if (Check):
        try:
            # Timer = Server.create_timer(0.5, Server.on_timer)
            # Server.JogJointmove = Server.create_subscription(JogJoint, '/findchessbot/cmd/move/jogjoint', Server.jogjoint_callback, 1)
            while rclpy.ok():
                rclpy.spin_once(Server)
                # if (Server.test == 10):
                #     Server.destroy_timer(Timer)
                #     Server.destroy_subscription(Timer)
                #     Timer = Server.create_timer(0.05, Server.on_timer)
                # else:
                #     print("JJJJJJJ")

        except KeyboardInterrupt:
            print('repeater stopped cleanly')
        except BaseException:
            print('exception in repeater:', file=sys.stderr)
            raise
        finally:
            Server.destroy_node()
            rclpy.shutdown() 
        
if __name__ == '__main__':
    main()
