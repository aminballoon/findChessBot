from __future__ import print_function
from ctypes import *
import chess
import copy
import chess.polyglot
from collections import Counter
import random
from IPython.display import display, clear_output
from timeit import default_timer as timer
import numpy as np
import multiprocessing
from itertools import product
from multiprocessing import Manager

class AI:
    def __init__(self, max_depth, timeMax, FEN = None):
        self.Fenbm = {}
        self.transposition = {}
        self.max_depth = max_depth
        self.timeMax = timeMax
        self.startTime = 0
        self.timeOutFlag = 0
        self.isWhite = 0
        self.gamma = 0.2
        if(FEN == None):
            self.s = chess.Board() # game state
        else:
            self.s = chess.Board(FEN)
        self.execute_value = {
                'P':+10,'p':-10,
                'N':+30,'n':-30,
                'B':+30,'b':-30,
                'R':+50,'r':-50,
                'Q':+90,'q':-90,
                'K':+900,'k':-900
            }
        # import NNUE
        self.nnue = cdll.LoadLibrary("./libnnueprobe.so")
        self.nnue.nnue_init(b"nn-62ef826d1a6d.nnue")
        # read opening book
        self.reader = chess.polyglot.open_reader('GMopenings.bin')

    def setFEN(self, FEN):
        self.s = chess.Board(FEN)

    def Matescore(self):
        if self.s.turn:
            if self.isWhite:
                return [-10000, None, False]
            else:
                return [10000, None, False]
        else:
            if self.isWhite:
                return [10000, None, False]
            else:
                return [-10000, None, False]

    def evaluation(self):
        fen = self.s.fen()
        try:
            eva = self.transposition[fen]
        except:
            eva = self.nnue.nnue_evaluate_fen(fen.encode())
            self.transposition[fen] = eva
        return eva

    def sortMoves(self,legal_moves):
        # Capture Heuristic & Last Best Move
        try:
            bm = [self.Fenbm[self.s.fen()]]
            legal_moves.remove(bm)
        except:
            bm = []
        movesGoodness = []
        unknownGoodness = []
        for c in legal_moves:
            KillerPiece = self.s.piece_at(chess.parse_square(str(c)[:2]))
            KilledPiece = self.s.piece_at(chess.parse_square(str(c)[2:4]))
            if(KilledPiece == None):
                unknownGoodness.append(c)
            else:
                val = self.execute_value[KillerPiece.symbol()] + self.execute_value[KilledPiece.symbol()]
                movesGoodness.append([c, val])
        # Shuffle UnknownGoodness for Heuristic
        random.shuffle(unknownGoodness)
        movesGoodness = sorted(movesGoodness, reverse = (1-self.isWhite), key=lambda x: x[1])
        knownGoodness = [x[0] for x in movesGoodness]
        return bm + knownGoodness + unknownGoodness

    def MaxPlayer(self, alpha, beta, depth):
        bestMove = None
        maxEva = -100000
        legal_moves = list(self.s.legal_moves)
        # Time Condition
        if(timer() - self.startTime > self.timeMax):
            self.timeOutFlag = 1
            eva = self.evaluation()
            return [eva, bestMove, True] # [eva, bm, isInterruptByTime]
        if(depth <= 0):
            eva = self.evaluation()
            return [eva, bestMove, False]
        if self.s.is_game_over():
            return self.Matescore()
        # Sort Moves
        sortedMoves = self.sortMoves(legal_moves)
        if(sortedMoves == []):
            eva = self.evaluation()
            return [eva, bestMove, False]
        for c in sortedMoves:
            self.s.push(c)
            eva, bm, isInterrupt = self.MinPlayer(alpha, beta, depth - 1)
            if(isInterrupt):
                self.s.pop()
                return [eva, bestMove, True]
            maxEva = max(maxEva,eva)
            alpha = max(alpha,eva)
            if(maxEva == eva):
                bestMove = c
            if(beta <= alpha):
                self.s.pop()
                break
            self.s.pop()
            if(bestMove != None):
                self.Fenbm[self.s.fen()] = bestMove
        return [maxEva, bestMove, False]
    
    def MinPlayer(self, alpha, beta, depth):
        bestMove = None
        minEva = 100000
        legal_moves = list(self.s.legal_moves)
        # Time Condition
        if(timer() - self.startTime > self.timeMax):
            self.timeOutFlag = 1
            eva = self.evaluation()
            return [eva, bestMove, True]
        if(depth <= 0):
            eva = self.evaluation()
            return [eva, bestMove, False]
        if self.s.is_game_over():
            return self.Matescore()
        # Sort Moves
        sortedMoves = self.sortMoves(legal_moves)
        if(sortedMoves == []):
            eva = self.evaluation()
            return [eva, bestMove, False]
        for c in sortedMoves:
            self.s.push(c)
            eva, bm, isInterrupt = self.MaxPlayer(alpha, beta, depth - 1)
            if(isInterrupt):
                self.s.pop()
                return [eva, bestMove, True]
            minEva = min(minEva,eva)
            beta = min(beta,eva)
            if(minEva == eva):
                bestMove = c
            if(beta <= alpha):
                self.s.pop()
                break
            self.s.pop()
            if(bestMove != None):
                self.Fenbm[self.s.fen()] = bestMove
        return [minEva, bestMove, False]
    
    def IterativeDeepening(self):
        self.startTime = timer()
        listOfResult = []
        # Iterative Deepening  
        for depth in range(2,self.max_depth,2):
            listOfResult.append(self.MaxPlayer(-100000,100000,depth))
            if(self.timeOutFlag):
                self.timeOutFlag = 0
                break
        l = len(listOfResult)
        print(listOfResult)
        while True:
            if(listOfResult[l-1][2] == False):
                bm = listOfResult[l-1][1]
                break
            l -= 1
        return bm
    
    def think(self):
        n = sum(1 for _ in self.reader.find_all(self.s))
        if n==0:
            print('book is empty, using AI ...')
            bm = self.IterativeDeepening()
            self.s.push(bm)
        else:
            for entry in self.reader.find_all(self.s):
                print('Found book moves:')
                print(entry.move)
                nextmove = self.s.san(entry.move)
                self.s.push_san(nextmove)
                break

ai = AI(max_depth = 6, timeMax = 45, FEN = None)