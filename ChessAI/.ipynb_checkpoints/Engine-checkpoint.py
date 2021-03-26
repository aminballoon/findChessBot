from __future__ import print_function
from ctypes import *
import chess
import copy
from collections import Counter
import random
from IPython.display import display, clear_output
from timeit import default_timer as timer
import numpy as np
from HelperFunction import beamSearch, sortMoves, evaluation, Opening
from SearchAlgorithm import MaxPlayer, MinPlayer, IterativeDeepening, Execute

transposition = {}
FENbm = {}
OpeningCache = []
timeMax = 3
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
    Execute()