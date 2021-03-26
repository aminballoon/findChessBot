from __future__ import print_function
from ctypes import *
import chess
import copy
from collections import Counter
import random
from IPython.display import display, clear_output
from timeit import default_timer as timer
import numpy as np
import time
import multiprocessing
from itertools import product
from multiprocessing import Manager

OpeningCache = []
timeMax = 250
timeOutFlag = 0

# import NNUE
dll_name = "libnnueprobe.so"
nnue = cdll.LoadLibrary("./libnnueprobe.so")
nnue.nnue_init(b"nn-62ef826d1a6d.nnue")
# import Opening Book
with open('OpeningBook.txt', 'r') as in_file:
    stripped = (line.strip() for line in in_file)
    lines = [line.split() for line in stripped if line]

execute_value = {
    'P':+10,'p':-10,
    'N':+30,'n':-30,
    'B':+30,'b':-30,
    'R':+50,'r':-50,
    'Q':+90,'q':-90,
    'K':+900,'k':-900
}

def beamSearch(s,legal_moves,isMin):
    ar = []
    moves = []
    for c in legal_moves:
        moves.append(c)
        s.push(c)
        ar.append(evaluation(s))
        s.pop() 
    a = abs(min(ar))
    j = [x + a for x in ar]
    b = sum(j)
    if(b == 0):
        return list(np.random.choice(moves, int(0.3* len(legal_moves)),replace = False)) 
    else:
        k = [x / b for x in j]
        if(isMin):
            k = [1-x for x in k]
        return list(np.random.choice(moves, int(0.3* len(legal_moves)), p=k,replace = False)) 
def sortMoves(s,legal_moves,isMin,Fenbm):
    # Capture Heuristic & Last Best Move
    try:
        bm = [Fenbm[s.fen()]]
        legal_moves.remove(bm)
    except:
        bm = []
    movesGoodness = []
    unknownGoodness = []
    for c in legal_moves:
        KillerPiece = s.piece_at(chess.parse_square(str(c)[:2]))
        KilledPiece = s.piece_at(chess.parse_square(str(c)[2:4]))
        if(KilledPiece == None):
            unknownGoodness.append(c)
        else:
            val = execute_value[KillerPiece.symbol()] + execute_value[KilledPiece.symbol()]
            movesGoodness.append([c, val])
    # Shuffle UnknownGoodness for Heuristic
    random.shuffle(unknownGoodness)
    movesGoodness = sorted(movesGoodness, reverse = isMin, key=lambda x: x[1])
    knownGoodness = [x[0] for x in movesGoodness]
    return bm + knownGoodness + unknownGoodness
def evaluation(s,transposition):
    fen = s.fen()
    try:
        eva = transposition[fen]
    except:
        eva = nnue.nnue_evaluate_fen(fen.encode())
        transposition[fen] = eva
    return eva
def Opening(OpeningCache):
    m = copy.copy([board.move_stack[-1]])
    board.pop()
    if(board.ply() == 0):
        for g in lines:
            if(g[board.ply()] == board.variation_san(m).split()[-1]):
                OpeningCache.append(g)
    else:
        tmp = []
        for op in OpeningCache:
            if(op[board.ply()] == board.variation_san(m).split()[-1]):
                tmp.append(op)
        OpeningCache = tmp 
    if(OpeningCache != []):
        # Count Probs
        mvs = [x[board.ply() + 1] for x in OpeningCache]
        san = list(Counter(mvs).keys())[list(Counter(mvs).values()).index(max(Counter(mvs).values()))]
        # Remove Silly Move
        OpeningCache = [x for x in OpeningCache if x[board.ply() + 1] == san]
        board.push(m[0])
        board.push_san(san)
    else:
        board.push(m[0])
    return OpeningCache

def MaxPlayer(s,alpha,beta,depth,startTime,initDepth,transposition,Fenbm,processCommunication):
    global timeOutFlag
    bestMove = None
    maxEva = -100000
    legal_moves = list(s.legal_moves)
    # Terminate Condition
    try:
        processCommunication[initDepth]
        return [eva, bestMove, True]
    except:
        pass
        
    if(timer() - startTime > timeMax):
        timeOutFlag = 1
        eva = evaluation(s,transposition)
        return [eva, bestMove, True]
    if(depth <= 0):
        eva = evaluation(s,transposition)
        return [eva, bestMove, False]
    if s.is_game_over():
        return [-10000, bestMove, False]
    # Sort Moves
    sortedMoves = sortMoves(s,legal_moves,0,Fenbm)
    #sortedMoves = beamSearch(s,legal_moves,0)
    if(sortedMoves == []):
        eva = evaluation(s,transposition)
        return [eva, bestMove, False]
    for c in sortedMoves:
        s.push(c)
        eva, bm, isInterrupt = MinPlayer(s,alpha,beta,depth - 1,startTime,initDepth,transposition,Fenbm,processCommunication)
        if(isInterrupt):
            return [eva, bestMove, True]
        maxEva = max(maxEva,eva)
        alpha = max(alpha,eva)
        if(maxEva == eva):
            bestMove = c
        if(beta <= alpha):
            s.pop()
            break
        s.pop()
        if(bestMove != None):
            Fenbm[s.fen()] = bestMove
    if(depth == initDepth and isInterrupt == False):
        processCommunication[depth] = bestMove
    return [maxEva, bestMove, False]

def MinPlayer(s,alpha,beta,depth,startTime,initDepth,transposition,Fenbm,processCommunication):
    global timeOutFlag
    bestMove = None
    minEva = 100000
    legal_moves = list(s.legal_moves)
    # Terminate Condition
    try:
        processCommunication[initDepth]
        return [eva, bestMove, True]
    except:
        pass
    
    if(timer() - startTime > timeMax):
        timeOutFlag = 1
        eva = evaluation(s,transposition)
        return [eva, bestMove, True]

    if(depth <= 0):
        eva = evaluation(s,transposition)
        return [eva, bestMove, False]
    if s.is_game_over():
        return [10000, bestMove, False]
    # Sort Moves
    sortedMoves = sortMoves(s,legal_moves,1,Fenbm)
    #sortedMoves = beamSearch(s,legal_moves,0)
    if(sortedMoves == []):
        eva = evaluation(s,transposition)
        return [eva, bestMove, False]
    for c in sortedMoves:
        s.push(c)
        eva, bm, isInterrupt = MaxPlayer(s,alpha,beta,depth - 1,startTime,initDepth,transposition,Fenbm,processCommunication)
        if(isInterrupt):
            return [eva, bestMove, True]
        minEva = min(minEva,eva)
        beta = min(beta,eva)
        if(minEva == eva):
            bestMove = c
        if(beta <= alpha):
            s.pop()
            break
        s.pop()
        if(bestMove != None):
            Fenbm[s.fen()] = bestMove
    if(depth == initDepth and isInterrupt == False):
        processCommunication[depth] = bestMove
    return [minEva, bestMove, False]
    
def IterativeDeepening(board,isWhite,transposition,Fenbm,processCommunication):
    global timeOutFlag
    start = timer()
    listOfResult = []
    # Iterative Deepening  
    st = time.time()
    for depth in range(2,max_depth):
        if(isWhite):
            listOfResult.append(MaxPlayer(board,-100000,100000,depth,start,depth,transposition,Fenbm,processCommunication))
        else:
            listOfResult.append(MinPlayer(board,-100000,100000,depth,start,depth,transposition,Fenbm,processCommunication))  
        if(timeOutFlag):
            timeOutFlag = 0
            break
    print(time.time() - st)
    print(listOfResult)
#     if(listOfResult[-1][0] > listOfResult[-2][0]):
#         return listOfResult[-1]
#     else:
#         return listOfResult[-2] 
    
def Execute(transposition,Fenbm,processCommunication):
    global OpeningCache
    if(OpeningCache != [] or board.ply() == 1):
        OpeningCache = Opening(OpeningCache)
        if(OpeningCache == []):
            value, bm = IterativeDeepening(board,0,transposition,Fenbm,processCommunication)
            board.push(bm)
    else:
        value, bm = IterativeDeepening(board,0,transposition,Fenbm,processCommunication)
        board.push(bm)
def parallelExecution(transposition,Fenbm,processCommunication):
    global OpeningCache
    if(OpeningCache != [] or board.ply() == 1):
        OpeningCache = Opening(OpeningCache)
        if(OpeningCache == []):
            value, bm = ParallelIterativeDeepening(board,0,transposition,Fenbm,processCommunication)
            board.push(bm)
    else:
        value, bm = ParallelIterativeDeepening(board,0,transposition,Fenbm,processCommunication)
        board.push(bm)

def ParallelIterativeDeepening(board,isWhite,transposition,Fenbm,processCommunication):
    start = timer()
    listOfResult = []
    # Iterative Deepening
    st = time.time()
    if(isWhite):
        with multiprocessing.Pool(multiprocessing.cpu_count()-1) as pool:   
            result = pool.starmap(MaxPlayer,zip(product([board],[-100000],[100000],[2,3,4,5],[start]),[2,3,4,5]))
    else:
        with multiprocessing.Pool(multiprocessing.cpu_count()-1) as pool:   
            result = pool.starmap(MinPlayer,zip([board]*5,[-100000]*5,[100000]*5,[2,2,2,4,4],[start]*5,[2,2,2,4,4],[transposition]*5,[Fenbm]*5,[processCommunication]*5))
    print(result)
    print(processCommunication)
    print(time.time() - st)

if __name__ == '__main__':
    manager = Manager()
    transposition = manager.dict()
    Fenbm = manager.dict()
    processCommunication = manager.dict()
    board = chess.Board()
    while True:
        if(board.is_game_over()):
            break
        max_depth = 6
        clear_output(wait=True)
        display(board)
        move = input("Input your move : ")
        try:
            board.push_uci(move)
        except:
            continue
        clear_output(wait=True)
        display(board)
        parallelExecution(transposition,Fenbm,processCommunication)
    