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



List = [[-0.262, 1.152], [-0.262, 1.167], [-0.261, 1.177], [-0.259, 1.19], [-0.259, 1.203], [-0.258, 1.217], [-0.256, 1.226], [-0.255, 1.239], [-0.255, 1.252], [-0.254, 1.265], [-0.252, 1.277], [-0.252, 1.293], [-0.253, 1.307], [-0.251, 1.319], [-0.252, 1.332], [-0.253, 1.349], [-0.251, 1.36], [-0.251, 1.372], [-0.252, 1.388], [-0.251, 1.4], [-0.252, 1.416], [-0.252, 1.432], [-0.253, 1.445], [-0.253, 1.46], [-0.253, 1.475], [-0.254, 1.487], [-0.256, 1.504], [-0.255, 1.518], [-0.256, 1.533], [-0.257, 1.549], [-0.258, 1.561], [-0.258, 1.576], [-0.262, 1.593], [-0.262, 1.608], [-0.262, 1.622], [-0.265, 1.638], [-0.267, 1.654], [-0.267, 1.669], [-0.269, 1.684], [-0.271, 1.7], [-0.273, 1.715], [-0.274, 1.728], [-0.276, 1.743], [-0.279, 1.76], [-0.28, 1.774], [-0.284, 1.791], [-0.285, 1.806], [-0.288, 1.822], [-0.291, 1.838], [-0.294, 1.854], [-0.297, 1.87], [-0.299, 1.883], [-0.302, 1.899], [-0.304, 1.915], [-0.309, 1.932], [-0.311, 1.945], [-0.315, 1.962], [-0.317, 1.977], [-0.322, 1.991], [-0.326, 2.008], [-0.33, 2.022], [-0.334, 2.039], [-0.338, 2.053], [-0.342, 2.069], [-0.348, 2.085], [-0.352, 2.099], [-0.355, 2.115], [-0.362, 2.13], [-0.368, 2.146], [-0.372, 2.159], [-0.378, 2.175], [-0.384, 2.19], [-0.391, 2.205], [-0.397, 2.22], [-0.403, 2.235], [-0.409, 2.25], [-0.419, 2.264], [-0.425, 2.279], [-0.431, 2.294], [-0.441, 2.308], [-0.447, 2.323], [-0.457, 2.337], [-0.468, 2.351], [-0.478, 2.365], [-0.484, 2.379], [-0.495, 2.393], [-0.505, 2.407], [-0.516, 2.42], [-0.528, 2.431], [-0.539, 2.445], [-0.554, 2.46], [-0.567, 2.471], [-0.579, 2.484], [-0.592, 2.495], [-0.61, 2.507], [-0.622, 2.52], [-0.64, 2.531], [-0.654, 2.542], [-0.673, 2.553], [-0.688, 2.563], [-0.709, 2.571], [-0.725, 2.581], [-0.746, 2.592], [-0.768, 2.6], [-0.79, 2.61], [-0.809, 2.616], [-0.832, 2.623], [-0.857, 2.63], [-0.882, 2.637], [-0.907, 2.643], [-0.927, 2.648], [-0.954, 2.654], [-0.981, 2.656], [-1.008, 2.661], [-1.035, 2.662], [-1.063, 2.667], [-1.091, 2.668], [-1.119, 2.668], [-1.147, 2.668], [-1.175, 2.668], [-1.203, 2.667], [-1.231, 2.666], [-1.258, 2.662], [-1.28, 2.66], [-1.306, 2.655], [-1.331, 2.65], [-1.357, 2.647], [-1.381, 2.641], [-1.405, 2.635], [-1.427, 2.629], [-1.45, 2.622], [-1.464, 2.613], [-1.485, 2.605], [-1.505, 2.598], [-1.518, 2.588], [-1.537, 2.58], [-1.552, 2.569], [-1.563, 2.559], [-1.577, 2.548], [-1.587, 2.538], [-1.6, 2.526], [-1.608, 2.516], [-1.62, 2.504], [-1.625, 2.491], [-1.632, 2.48], [-1.639, 2.465], [-1.645, 2.454], [-1.648, 2.441], [-1.651, 2.427], [-1.656, 2.416], [-1.658, 2.403], [-1.66, 2.389], [-1.662, 2.375], [-1.66, 2.359], [-1.659, 2.347], [-1.659, 2.333], [-1.657, 2.317], [-1.655, 2.304], [-1.652, 2.288], [-1.65, 2.275], [-1.645, 2.26], [-1.644, 2.246], [-1.639, 2.231], [-1.634, 2.216], [-1.628, 2.201], [-1.623, 2.186], [-1.617, 2.17], [-1.611, 2.155], [-1.604, 2.142], [-1.598, 2.126], [-1.588, 2.11], [-1.582, 2.094], [-1.575, 2.081], [-1.566, 2.062], [-1.559, 2.048], [-1.549, 2.032], [-1.541, 2.018], [-1.531, 2.001], [-1.524, 1.987], [-1.512, 1.972], [-1.502, 1.954], [-1.494, 1.942], [-1.484, 1.925], [-1.472, 1.909], [-1.461, 1.893], [-1.454, 1.879], [-1.443, 1.863], [-1.431, 1.847], [-1.42, 1.831], [-1.408, 1.816], [-1.397, 1.8], [-1.386, 1.783], [-1.376, 1.771], [-1.364, 1.757], [-1.353, 1.74], [-1.341, 1.724], [-1.329, 1.709], [-1.317, 1.694], [-1.305, 1.678], [-1.293, 1.662], [-1.28, 1.648], [-1.268, 1.632], [-1.255, 1.617], [-1.243, 1.601], [-1.23, 1.586], [-1.22, 1.574], [-1.207, 1.559], [-1.194, 1.544], [-1.181, 1.528], [-1.168, 1.512], [-1.155, 1.496], [-1.145, 1.484], [-1.131, 1.469], [-1.117, 1.454], [-1.106, 1.441], [-1.092, 1.426], [-1.078, 1.41], [-1.067, 1.398], [-1.053, 1.382], [-1.041, 1.37], [-1.03, 1.357], [-1.016, 1.341], [-1.003, 1.329], [-0.991, 1.316], [-0.976, 1.3], [-0.964, 1.287], [-0.952, 1.274], [-0.939, 1.262], [-0.926, 1.249], [-0.913, 1.236], [-0.9, 1.223], [-0.887, 1.21], [-0.877, 1.2], [-0.864, 1.186], [-0.85, 1.174], [-0.839, 1.164], [-0.825, 1.15], [-0.814, 1.14], [-0.8, 1.126], [-0.789, 1.115], [-0.777, 1.106], [-0.765, 1.096], [-0.753, 1.086], [-0.741, 1.075], [-0.729, 1.065], [-0.717, 1.054], [-0.705, 1.043], [-0.692, 1.031], [-0.683, 1.025], [-0.669, 1.014], [-0.659, 1.007], [-0.65, 1.0], [-0.636, 0.988], [-0.626, 0.981], [-0.614, 0.974], [-0.604, 0.966], [-0.593, 0.958], [-0.585, 0.956], [-0.574, 0.948], [-0.562, 0.94], [-0.554, 0.936], [-0.543, 0.928], [-0.533, 0.925], [-0.524, 0.921], [-0.515, 0.918], [-0.507, 0.914], [-0.498, 0.91], [-0.488, 0.905], [-0.478, 0.901], [-0.472, 0.902], [-0.462, 0.897], [-0.455, 0.898], [-0.445, 0.893], [-0.438, 0.893], [-0.43, 0.894], [-0.424, 0.894], [-0.417, 0.894], [-0.409, 0.893], [-0.402, 0.893], [-0.397, 0.898], [-0.389, 0.897], [-0.384, 0.902], [-0.377, 0.901], [-0.372, 0.905], [-0.366, 0.91], [-0.36, 0.914], [-0.355, 0.917], [-0.349, 0.921], [-0.344, 0.925], [-0.341, 0.934], [-0.334, 0.936], [-0.331, 0.945], [-0.326, 0.948], [-0.322, 0.956], [-0.317, 0.963], [-0.312, 0.966], [-0.308, 0.973], [-0.307, 0.986], [-0.302, 0.993], [-0.299, 1.0], [-0.296, 1.007], [-0.293, 1.018], [-0.289, 1.025], [-0.288, 1.037], [-0.284, 1.043], [-0.282, 1.054], [-0.278, 1.064], [-0.276, 1.074], [-0.276, 1.086], [-0.273, 1.096], [-0.271, 1.106], [-0.268, 1.115], [-0.267, 1.13], [-0.266, 1.14], [-0.262, 1.149]]
        