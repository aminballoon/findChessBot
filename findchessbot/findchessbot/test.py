import numpy.matlib as matrix

from math import sqrt ,sin ,cos ,atan2, pi, degrees, radians
import numpy as np
import matplotlib.pyplot as plt
def Coorchess_to_point(Initial, Goal):
    Dict = {"A": -3.5, "B": -2.5,"C": -1.5, "D": -0.5, 
            "H": 3.5, "G": 2.5,"F": 1.5,"E": 0.5,
            "1": -3.5, "2": -2.5,"3": -1.5, "4": -0.5, 
            "8": 3.5, "7": 2.5,"6": 1.5,"5": 0.5}

    Initial_Pose = [round(num, 3) for num in [(.05*Dict[Initial[0]])+.745, .05*Dict[Initial[1]]]]
    Goal_Pose = [round(num, 3) for num in [(.05*Dict[Goal[0]])+.745, .05*Dict[Goal[1]]]]
    
    return Initial_Pose,Goal_Pose

def Kuy(Coor):
    Dict = {"A": -3.5,  "B": -2.5,  "C": -1.5,  "D": -0.5, 
            "H": 3.5,   "G": 2.5,   "F": 1.5,   "E": 0.5,
            "1": -3.5,  "2": -2.5,  "3": -1.5,  "4": -0.5, 
            "8": 3.5,   "7": 2.5,   "6": 1.5,   "5": 0.5}

    Pose = [round(num, 3) for num in [(.05*Dict[Coor[0]]), .05*Dict[Coor[1]]]]
    Theta = round(radians((degrees(atan2(Pose[1],Pose[0]))+270) % 360),3)
    r = round(sqrt((Pose[0] * Pose[0]) + (Pose[1] * Pose[1])),3)
    return ('\''+Coor+'\'' +' : '+ str([r,Theta]) + ',')


    
# for i in ['A','B','C','D','E','F','G','H']:
#     for j in ['1','2','3','4','5','6','7','8']:
#         print(Kuy(i+j))



Coorchess_to_point = {  'A1' : [0.247, 2.356],  'A2' : [0.215, 2.191],  'A3' : [0.19, 1.976],   'A4' : [0.177, 1.713],
                        'A5' : [0.177, 1.429],  'A6' : [0.19, 1.166],   'A7' : [0.215, 0.951],  'A8' : [0.247, 0.785],
                        'B1' : [0.215, 2.521],  'B2' : [0.177, 2.356],  'B3' : [0.146, 2.111],  'B4' : [0.127, 1.768],
                        'B5' : [0.127, 1.373],  'B6' : [0.146, 1.03],   'B7' : [0.177, 0.785],  'B8' : [0.215, 0.62],
                        'C1' : [0.19, 2.737],   'C2' : [0.146, 2.601],  'C3' : [0.106, 2.356],  'C4' : [0.079, 1.893],
                        'C5' : [0.079, 1.249],  'C6' : [0.106, 0.785],  'C7' : [0.146, 0.54],   'C8' : [0.19, 0.405],
                        'D1' : [0.177, 3.0],    'D2' : [0.127, 2.944],  'D3' : [0.079, 2.82],   'D4' : [0.035, 2.356],
                        'D5' : [0.035, 0.785],  'D6' : [0.079, 0.322],  'D7' : [0.127, 0.197],  'D8' : [0.177, 0.142],
                        'E1' : [0.177, 3.283],  'E2' : [0.127, 3.339],  'E3' : [0.079, 3.463],  'E4' : [0.035, 3.927],
                        'E5' : [0.035, 5.498],  'E6' : [0.079, 5.961],  'E7' : [0.127, 6.086],  'E8' : [0.177, 6.141],
                        'F1' : [0.19, 3.546],   'F2' : [0.146, 3.682],  'F3' : [0.106, 3.927],  'F4' : [0.079, 4.391],
                        'F5' : [0.079, 5.034],  'F6' : [0.106, 5.498],  'F7' : [0.146, 5.743],  'F8' : [0.19, 5.878],
                        'G1' : [0.215, 3.762],  'G2' : [0.177, 3.927],  'G3' : [0.146, 4.172],  'G4' : [0.127, 4.515],
                        'G5' : [0.127, 4.91],   'G6' : [0.146, 5.253],  'G7' : [0.177, 5.498],  'G8' : [0.215, 5.663],
                        'H1' : [0.247, 3.927],  'H2' : [0.215, 4.092],  'H3' : [0.19, 4.307],   'H4' : [0.177, 4.57],
                        'H5' : [0.177, 4.854],  'H6' : [0.19, 5.117],   'H7' : [0.215, 5.333],  'H8' : [0.247, 5.498], }


# fig, axs = plt.subplots(1)
# for i in np.arange(0, pi*2, 0.1):
#     r = 247
#     theta = (2.356 + i)%pi*2

# print(len(np.arange(0, pi*2, 0.02)))


x = np.arange(0.475, 0.625, 0.001)
y = np.arange(0.050, 0.200, 0.001)
print(x)
xnew = [round(num, 3) for num in x]
ynew = [round(num, 3) for num in y]

# list([x,y] for i in )
List = []
for i,j in zip(xnew,ynew):
    List.append([i,j])
    # if i == 2:
    #     break
for i,j in zip(xnew[::-1],ynew[::-1]):
    List.append([i,j])

print(List)
print(len(List))