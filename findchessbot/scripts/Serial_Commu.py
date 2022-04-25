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
        self.chess_squares = {  'a1':  0, 'a2':  1, 'a3':  2, 'a4':  3, 'a5':  4, 'a6':  5, 'a7':  6, 'a8':  7, 
                                'b1':  8, 'b2':  9, 'b3': 10, 'b4': 11, 'b5': 12, 'b6': 13, 'b7': 14, 'b8': 15, 
                                'c1': 16, 'c2': 17, 'c3': 18, 'c4': 19, 'c5': 20, 'c6': 21, 'c7': 22, 'c8': 23, 
                                'd1': 24, 'd2': 25, 'd3': 26, 'd4': 27, 'd5': 28, 'd6': 29, 'd7': 30, 'd8': 31, 
                                'e1': 32, 'e2': 33, 'e3': 34, 'e4': 35, 'e5': 36, 'e6': 37, 'e7': 38, 'e8': 39, 
                                'f1': 40, 'f2': 41, 'f3': 42, 'f4': 43, 'f5': 44, 'f6': 45, 'f7': 46, 'f8': 47, 
                                'g1': 48, 'g2': 49, 'g3': 50, 'g4': 51, 'g5': 52, 'g6': 53, 'g7': 54, 'g8': 55, 
                                'h1': 56, 'h2': 57, 'h3': 58, 'h4': 59, 'h5': 60, 'h6': 61, 'h7': 62, 'h8': 63}
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
        self.Casting = self.create_service(Castling, '/findchessbot/casting',self.casting_callback)
        self.PicknPlace = self.create_service(PicknPlace, '/findchessbot/picknplace',self.picknplace_callback)
        self.PromotePawn = self.create_service(PromotePawn, '/findchessbot/promotepawn',self.promotepawn_callback)
        self.crc16 = CRC16()

    
    def mcu_mode_srv_callback(self, request, response):
        New_Mode = request.update_mode
        if (self.Server_MCU_Mode != 0):
            # Check State Mbed
            if(self.MCU_Mode != 0):
                response.result = False
            else:
                response.result = True
        else:
            response.result = True
        if (response.result):
            self.Server_MCU_Mode = New_Mode
            self.MCU_Mode = New_Mode
            self.get_logger().info("Chage Mode Done!!")
        else:
            self.get_logger().info("Failed to Chage Mode")
        # print(request.position_x, request.position_y, request.position_z, request.orientation_yaw)
        return response

    def Mode1(self,Command): # Mode1 = Control State
        frame1 = [0x86,self.chess_squares[Command],2]
        data = frame1 + self.crc16.calculate(frame1)
        self.get_logger().info(str(Command) + " : " + data)
        # H7.write(data1)
        # time.sleep(0.1)
        print(data)

    def Mode2(self,Command): # Mode2 = Jog Joint State
        print("") 

    def Mode3(self,Command): # Mode3 = Linear Joint State
        print("")

    def on_timer(self):
        self.test += 1
        print("qqqqq")
        pass

    def jogjoint_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    def capture_callback(self, request, response):
        response.result = True
        print("capture_callback")
        print(request)
        self.Mode1(request.chess_square_eat)
        self.Mode1(request.chess_square_pick)
        self.Mode1(request.chess_square_place)

        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def casting_callback(self, request, response):
        response.result = True
        print("castling_callback")
        print(request)
        self.Mode1(request.chess_square_pick_king)
        self.Mode1(request.chess_square_place_king)
        self.Mode1(request.chess_square_pick_rook)
        self.Mode1(request.chess_square_place_rook)


        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def picknplace_callback(self, request, response):
        response.result = True
        print("picknplace_callback")
        print(request)
        print("\n\n")
        self.Mode1(request.chess_square_pick)
        self.Mode1(request.chess_square_place)

        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

    def promotepawn_callback(self, request, response):
        response.result = True
        print("promotepawn_callback")
        print(request)
        print("\n\n")
        self.Mode1(request.chess_square_pick_pawn)
        self.Mode1(request.chess_square_pawn_to_trash)
        self.Mode1(request.chess_square_pick_queen)
        self.Mode1(request.chess_square_place)

        # self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response


        
def main(args=None):
    rclpy.init(args=args)
    Server = MCU_Server()
    Server.get_logger().info("Serial Comunication OK!!")
    
    try:
        # H7 = serial.Serial('/dev/ttyACM0', 115200, timeout=5)
        # crc16 = CRC16()
        print("Test")
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
