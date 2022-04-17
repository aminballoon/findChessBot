from __future__ import print_function
from ctypes import *
import chess
import chess.polyglot
import chess.svg
import random
from timeit import default_timer as timer

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
        self.isEndgame = 0
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
        self.nnue.nnue_init(b"gek-net.bin")
        # read opening book
        self.reader = chess.polyglot.open_reader('GMopenings.bin')
        # set isWhite
        self.setisWhite()

    def setisWhite(self):
        if(self.s.fen().split()[1] == 'w'):
            self.isWhite = 1
        else:
            self.isWhite = 0

    def setFEN(self, FEN):
        self.s = chess.Board(FEN)

    def setState(self, state):
        self.s = state
        self.isWhite = state.turn

    def getSVG(self, size=350):
        return chess.svg.board(self.s, size=size)

    def endgameScore(self, c):
        if(self.isEndgame):
            score = 0
            piece = self.s.piece_at(chess.parse_square(str(c)[2:4]))
            if(piece != None):
                score += self.execute_value[piece.symbol()]
            if(len(str(c)) != 4):
                score += self.execute_value[str(c)[-1].upper()]
        else:
            score = 0
        if(self.isWhite):
            score = -score
        return score

    def materialCheck(self):
        black_mat = 0
        white_mat = 0
        for sq in chess.SQUARES:
            piece = self.s.piece_at(sq)
            if(piece != None):
                if(piece.symbol().islower()):
                    black_mat += 1
                else:
                    white_mat += 1
        return white_mat, black_mat

    def isEndgameCheck(self):
        white_mat, black_mat = self.materialCheck()
        if(black_mat < 13 or white_mat < 13):
            print("This is Endgame !")
            self.isEndgame = 1
        else:
            print("This is Middle ~~")
            self.isEndgame = 0

    def gameOverScore(self):
        if self.s.is_checkmate():
            return self.Matescore()
        else:
            # Stalemate or insufficient mat
            # check material
            white_mat, black_mat = self.materialCheck()
            willWhitewin = (white_mat - black_mat) > 3
            if self.s.turn: # White Turn
                if self.isWhite: # I am White
                    if willWhitewin: # White will win but stalemate !
                        return [-1000000, None, False]
                    else:
                        return [1000000, None, False]
                else:
                    if willWhitewin: # White will win but stalemate !
                        return [1000000, None, False]
                    else:
                        return [-1000000, None, False]
            else:
                if self.isWhite: # I am White
                    if willWhitewin: # White will win but stalemate !
                        return [1000000, None, False]
                    else:
                        return [-1000000, None, False]
                else:
                    if willWhitewin: # White will win but stalemate !
                        return [-1000000, None, False]
                    else:
                        return [1000000, None, False]

    def Matescore(self):
        if self.s.turn:
            if self.isWhite:
                return [-1000000, None, False]
            else:
                return [1000000, None, False]
        else:
            if self.isWhite:
                return [1000000, None, False]
            else:
                return [-1000000, None, False]

    def evaluation(self):
        fen = self.s.fen()
        sp = fen.split()
        if self.isWhite:
            sp[1] = 'w'
        else:
            sp[1] = 'b'
        fen = " ".join(sp)
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

    def MaxPlayer(self, alpha, beta, depth, acceva = 0):
        bestMove = None
        mateMove = None
        maxEva = -100000
        legal_moves = list(self.s.legal_moves)
        # Time Condition
        if(timer() - self.startTime > self.timeMax):
            self.timeOutFlag = 1
            eva = acceva + self.evaluation()
            return [eva, bestMove, True] # [eva, bm, isInterruptByTime]
        if(depth <= 0):
            eva = acceva + self.evaluation()
            return [eva, bestMove, False]
        if self.s.is_game_over():
            return self.gameOverScore()
        # Sort Moves
        sortedMoves = self.sortMoves(legal_moves)
        if(sortedMoves == []):
            eva = acceva + self.evaluation()
            return [eva, bestMove, False]
        for c in sortedMoves:
            sc = self.endgameScore(c)
            self.s.push(c)
            nnue_sc = self.evaluation()
            eva, bm, isInterrupt = self.MinPlayer(alpha, beta, depth - 1, acceva + nnue_sc + abs(self.gamma*nnue_sc)*sc)
            if(eva == -1000000):
                mateMove = c
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
            else:
                bestMove = mateMove
        return [maxEva, bestMove, False]
    
    def MinPlayer(self, alpha, beta, depth, acceva = 0):
        bestMove = None
        mateMove = None
        minEva = 100000
        legal_moves = list(self.s.legal_moves)
        # Time Condition
        if(timer() - self.startTime > self.timeMax):
            self.timeOutFlag = 1
            eva = acceva + self.evaluation()
            return [eva, bestMove, True]
        if(depth <= 0):
            eva = acceva + self.evaluation()
            return [eva, bestMove, False]
        if self.s.is_game_over():
            return self.gameOverScore()
        # Sort Moves
        sortedMoves = self.sortMoves(legal_moves)
        if(sortedMoves == []):
            eva = acceva + self.evaluation()
            return [eva, bestMove, False]
        for c in sortedMoves:
            sc = self.endgameScore(c)
            self.s.push(c)
            nnue_sc = self.evaluation()
            eva, bm, isInterrupt = self.MaxPlayer(alpha, beta, depth - 1, acceva + nnue_sc + abs(self.gamma*nnue_sc)*sc)
            if(eva == -1000000):
                mateMove = c
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
            else:
                bestMove = mateMove
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
            if(listOfResult[-1][0] == 1000000): # Fastest Mate
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
        self.setisWhite()
        self.isEndgameCheck()
        n = sum(1 for _ in self.reader.find_all(self.s))
        if n==0:
            print('book is empty, using AI ...')
            bm = self.IterativeDeepening()
            bm_san = self.s.san(bm)
            print(bm_san)
            promotePiece = None
            if 'x' in bm_san:
                flag = 1 # Capture piece
            elif '=' in bm_san:
                flag = 2 # Promote pawn
                promotePiece = bm_san.split('=')[1]
            else:
                flag = 0 # Normal move
            self.s.push(bm)
            return bm.uci(), flag, promotePiece
        else:
            for entry in self.reader.find_all(self.s):
                print('Found book moves:')
                print(entry.move)
                nextmove = self.s.san(entry.move)
                promotePiece = None
                if 'x' in nextmove:
                    flag = 1 # Capture piece
                elif '=' in nextmove:
                    flag = 2 # Promote pawn
                    promotePiece = nextmove.split('=')[1]
                else:
                    flag = 0 # Normal move
                self.s.push_san(nextmove)
                return entry.move.uci(), flag, promotePiece
                break

'''
[Example Program]
flag {0: Normal move, 1: Capture piece, 2: Promote pawn}
'''

ai = AI(max_depth = 6, timeMax = 45, FEN = None)

while True:
    m = input("move : ")
    try:
        ai.s.push_uci(m)
    except:
        print("Game State can't reach your input")
        continue
    print("------------------------")
    print(ai.s)
    uci_ai, flag, promotePiece = ai.think()
    print("------------------------")
    print("AI Move : {}".format(uci_ai))
    print("AI Flag : {}".format(flag))
    print("AI PromotePiece : {}".format(promotePiece))
    print(ai.s)

    # Save SVG
    # f = open("haha.svg", "w")
    # f.write(ai.getSVG())
    # f.close()



    
    
