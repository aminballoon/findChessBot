#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from Utils.RecognitionUtils import completePipeline
from Utils.ChessAIUtils import AI
import cv2
import chess

import sys
sys.path.insert(0,'../../')
from findchessbot.srv import Capture
from findchessbot.srv import Castling
from findchessbot.srv import MoveChesspiece
from findchessbot.srv import PromotePawn
sys.path.insert(0,'.')
# Need to Set before playing
AISide = chess.WHITE

class AiClientAsync(Node):

    def __init__(self):
        super().__init__('ai_client_async')
        self.cli = self.create_client(Capture, 'capture_service')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.req = Capture.Request()

    def send_capture_request(self, chess_square_pick, chess_square_eat):
        self.req.chess_square_eat = chess_square_eat
        self.req.chess_square_pick = chess_square_pick
        self.req.chess_square_place = chess_square_eat
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('sent_capture_request')

    def send_castling_request(self, is_king_side):
        if AISide == chess.WHITE:
            row = 1
        else:
            row = 8
        if is_king_side == True:
            self.req.chess_square_pick_king = 'e' + row
            self.req.chess_square_place_king = 'g' + row
            self.req.chess_square_pick_rook = 'h'+ row
            self.req.chess_square_place_rook = 'f' + row
        else:
            self.req.chess_square_pick_king = 'e' + row
            self.req.chess_square_place_king = 'c' + row
            self.req.chess_square_pick_rook = 'a' + row
            self.req.chess_square_place_rook = 'd' + row
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('sent_castling_request')

    def send_move_chess_piece_request(self, chess_square_pick, chess_square_place):
        self.req.chess_square_pick = chess_square_pick
        self.req.chess_square_place = chess_square_place
        self.future = self.cli.call_async(self.req)
        self.get_logger().info('sent_move_chess_piece_request')

    def send_promote_pawn_request(self, chess_square_pawn, chess_square_place):
        self.req.chess_square_pawn = chess_square_pawn
        self.req.chess_square_trash = '99'
        self.req.chess_square_pick_queen = '00'
        self.req.chess_square_place = chess_square_place
        self.get_logger().info('sent_promote_pawn_request_request')

def main(args=None):
    rclpy.init(args=args)

    ai_client = AiClientAsync()
    cam = cv2.VideoCapture(2)
    while True:
        input('[Enter] To Trigger The Program')
        # rawImgPath = '/home/trapoom555/Desktop/ChessDetection/Data/WIN_20210326_10_21_31_Pro.jpg'
        # img = cv2.imread(rawImgPath)

        ret, img = cam.read()
        cv2.imshow('preview', img)
        cv2.waitKey(0)
        cam.release()
        cv2.destroyAllWindows()
        b, dst = completePipeline(img)
        b.turn = AISide
        print("ANSSSSSS")
        print(b)
        print(b.fen())


        confirm = input("Is the Game State correct ? [Y/n] : ")

        if(confirm == 'Y' or confirm == 'yes' or confirm == 'y'):
            ai = AI(max_depth = 6, timeMax = 45, FEN = b.fen())
            uci_ai, flag, promotePiece = ai.think()
            print(uci_ai)
        elif(confirm == 'N' or confirm == 'no' or confirm == 'n'):
            fen = input("Please Input the Correct FEN : ")
            ai = AI(max_depth = 6, timeMax = 45, FEN = fen)
            uci_ai, flag, promotePiece = ai.think()
            print(uci_ai)
        else:
            continue

        ''' WRITING SRV TO ROS '''
        ''' flag {0: Normal move, 1: Capture piece, 2: Promote pawn, 3: King side Castling, 4: Queen side Castling} '''
        if flag == 0:
            ai_client.send_move_chess_piece_request(uci_ai[:2], uci_ai[:2])
        elif flag == 1:
            ai_client.send_capture_request(uci_ai[:2], uci_ai[:2])
        elif flag == 2:
            ai_client.send_promote_pawn_request(uci_ai[:2], uci_ai[:2])
        elif flag == 3:
            ai_client.send_castling_request(True)
        elif flag == 4:
            ai_client.send_castling_request(False)

if __name__ == '__main__':
    main()
