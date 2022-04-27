#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from Utils.RecognitionUtils import completePipeline, RecurrentRecognitionFunction, calculateSOG
from Utils.ChessAIUtils import AI
import cv2
import chess

import time

import sys
sys.path.insert(0,'../../')
from findchessbot.srv import Capture
from findchessbot.srv import Castling
from findchessbot.srv import PicknPlace
from findchessbot.srv import PromotePawn
from ai.srv import ForceRecognize
from ai.srv import AIComputeAndPlay
from ai.srv import RecurrentRecognition
from ai.srv import CaptureBeforeHumanPlay

sys.path.insert(0,'.')
sys.path.insert(0,'./Utils')
# Need to Set before playing
AISide = chess.WHITE

class AiClientAsync(Node):

    def __init__(self):
        super().__init__('ai_client_async')
        self.Capture = self.create_client(Capture, '/findchessbot/capture')
        self.Castling = self.create_client(Castling, '/findchessbot/castling')
        self.PickNPlace = self.create_client(PicknPlace, '/findchessbot/picknplace')
        self.PromotePawn = self.create_client(PromotePawn, '/findchessbot/promotepawn')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        self.CaptureReq = Capture.Request()
        self.CastlingReq = Castling.Request()
        self.PickNPlaceReq = PicknPlace.Request()
        self.PromotePawnReq = PromotePawn.Request()

    def send_capture_request(self, chess_square_pick, chess_square_eat):
        self.CaptureReq.chess_square_eat = chess_square_eat
        self.CaptureReq.chess_square_pick = chess_square_pick
        self.CaptureReq.chess_square_place = chess_square_eat
        self.future = self.Capture.call_async(self.CaptureReq)
        self.get_logger().info('sent_capture_request')

    def send_castling_request(self, is_king_side):
        if AISide == chess.WHITE:
            row = '1'
        else:
            row = '8'
        if is_king_side == True:
            self.CastlingReq.chess_square_pick_king = 'e' + row
            self.CastlingReq.chess_square_place_king = 'g' + row
            self.CastlingReq.chess_square_pick_rook = 'h'+ row
            self.CastlingReq.chess_square_place_rook = 'f' + row
        else:
            self.CastlingReq.chess_square_pick_king = 'e' + row
            self.CastlingReq.chess_square_place_king = 'c' + row
            self.CastlingReq.chess_square_pick_rook = 'a' + row
            self.CastlingReq.chess_square_place_rook = 'd' + row
        self.future = self.Castling.call_async(self.CastlingReq)
        self.get_logger().info('sent_castling_request')

    def send_pick_n_place_request(self, chess_square_pick, chess_square_place):
        self.PickNPlaceReq.chess_square_pick = chess_square_pick
        self.PickNPlaceReq.chess_square_place = chess_square_place
        self.future = self.PickNPlace.call_async(self.PickNPlaceReq)
        self.get_logger().info('sent_move_chess_piece_request')

    def send_promote_pawn_request(self, chess_square_pawn, chess_square_place):
        self.PromotePawnReq.chess_square_pick_pawn = chess_square_pawn
        self.PromotePawnReq.chess_square_pawn_to_trash = '99'
        self.PromotePawnReq.chess_square_pick_queen = '00'
        self.PromotePawnReq.chess_square_place = chess_square_place
        self.future = self.PromotePawn.call_async(self.PromotePawnReq)
        self.get_logger().info('sent_promote_pawn_request_request')

class AiServer(Node):
    def __init__(self):
        super().__init__('ai_server')
        # Argument | Return
        self.forceRecognize = self.create_service(ForceRecognize, '/ai/ForceRecognize', self.forceRecognize_callback) # None | FEN
        self.aiComputeAndPlay = self.create_service(AIComputeAndPlay, '/ai/AIComputeAndPlay', self.aiComputeAndPlay_callback) # FEN | uci_move
        self.recurrentRecognition = self.create_service(RecurrentRecognition, '/ai/RecurrentRecognition', self.recurrentRecognition_callback) # FEN | uci_move
        self.captureBeforeHumanPlay = self.create_service(CaptureBeforeHumanPlay, '/ai/CaptureBeforeHumanPlay', self.captureBeforeHumanPlay_callback) # FEN | uci_move

        self.dict_grad = None

    def forceRecognize_callback(self, request, response):
        try:
            # cam = cv2.VideoCapture(2)
            # print("[Waiting] Program")
            # ret, img = cam.read()
            # cam.release()
            # time.sleep(1.0)
            # print("[Ready] Program")

            # Capture
            cam = cv2.VideoCapture(2)
            ret, img = cam.read()
            # cv2.imshow('preview', img)
            # cv2.waitKey(0)
            # cam.release()
            # cv2.destroyAllWindows()
            b, dst = completePipeline(img)
            b.turn = AISide

            print(b)
            response.fen_out = b.fen()
            response.is_success = True
        except:
            response.is_success = False
        return response

    def aiComputeAndPlay_callback(self, request, response):
        try:
            ai_client = AiClientAsync()
            ai = AI(max_depth = 6, timeMax = 45, FEN = request.fen_in)
            uci_ai, flag, promotePiece = ai.think()

            response.uci_move = uci_ai

            ''' WRITING SRV TO ROS '''
            ''' flag {0: Normal move, 1: Capture piece, 2: Promote pawn, 3: King side Castling, 4: Queen side Castling} '''
            if flag == 0:
                ai_client.send_pick_n_place_request(uci_ai[:2], uci_ai[2:])
            elif flag == 1:
                ai_client.send_capture_request(uci_ai[:2], uci_ai[2:])
            elif flag == 2:
                ai_client.send_promote_pawn_request(uci_ai[:2], uci_ai[2:])
            elif flag == 3:
                ai_client.send_castling_request(True)
            elif flag == 4:
                ai_client.send_castling_request(False)

            response.is_success = True
        except Exception as e:
            print(e)
            response.is_success = False

        return response

    def captureBeforeHumanPlay_callback(self, request, response):
        try:
            cam = cv2.VideoCapture(2)
            ret, img = cam.read()
            cam.release()
            dict_grad = calculateSOG(img)
            print(dict_grad)
            self.dict_grad = dict_grad
            response.is_success = True
        except Exception as e:
            print(e)
            response.is_success = False
        return response

    def recurrentRecognition_callback(self, request, response):
        try:
            cam = cv2.VideoCapture(2)
            ret, img = cam.read()
            cam.release()
            fen = RecurrentRecognitionFunction(img, self.dict_grad, request.prev_fen_in)
            response.fen_out = fen
            response.is_success = True
        except Exception as e:
            print(e)
            response.is_success = False
        return response



def main(args=None):
    rclpy.init(args=args)
    Server = AiServer()
    Server.get_logger().info("Server Started")
    try:
        while rclpy.ok():
            rclpy.spin_once(Server)

    except KeyboardInterrupt:
        print('repeater stopped cleanly')
    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise
    finally:
        Server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    cam = cv2.VideoCapture(2)
    print("[Waiting] Program")
    ret, img = cam.read()
    cam.release()
    time.sleep(1.0)
    print("[Ready] Program")
    main()
