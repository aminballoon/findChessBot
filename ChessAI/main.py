from AIacc import ai
from timeit import default_timer as timer
if __name__ == "__main__":
    ai.setFEN(FEN = "7R/8/8/8/3kp3/8/r7/4K3 w - - 0 1")
    start = timer()
    ai.think()
    print(timer() - start)
    print(ai.s)
    # print(ai.s)
    # while True:
    #     a = input("input move : ")
    #     ai.s.push_uci(a)
    #     print(ai.s)
    #     ai.think()
    #     print(ai.s)