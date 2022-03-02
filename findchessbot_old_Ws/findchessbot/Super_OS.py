from math import sqrt ,sin ,cos ,atan2, pi
import numpy as np
class findchessbot():
    def __init__(self):
        self.L1 = 0.013245
        self.L2 = 0.370
        self.L3 = 0.315
        self.L12 = 0.383245
        self.h1 = 0.125 
        self.h3 = 0.065
        self.h4 = 0.190

        self.C1 = 1
        self.C13 = 1
        self.C134 = 1
        self.C3 = 1
        self.S1 = 0
        self.S13 = 0
        self.S134 = 0
        self.S3 = 1

        self.q1 = 0
        self.q2 = 0
        self.q3 = 0
        self.q4 = 0

        self.pose_X = 0.745
        self.pose_Y = 0
        self.pose_Z = 0
        self.pose_Yaw = 0

        self.H1, self.H2, self.H3, self.H4, self.He = self.FK(q1=0, q2=0, q3=0, q4=0, init=1)

        self.Je, self.Je_star, self.Je_inv = self.Jacobian(init = 1)
        self.Coorchess_to_point = {  'A1' : [0.247, 2.356],  'A2' : [0.215, 2.191],  'A3' : [0.19, 1.976],   'A4' : [0.177, 1.713],
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

    def IK(self,X,Y,Z,Yaw):
        C3 = ((X*X) + (Y*Y) - (pow(self.L12,2)) - (pow(self.L3,2)) ) / (2*self.L12*self.L3)
        S3 = sqrt(1-pow(C3,2))
        q3 = atan2(S3,C3)
        L3s3 = self.L3*S3
        L123c3 = self.L12+(self.L3*C3)
        S1 = (-L3s3*X) + (L123c3*Y)
        C1 = (L3s3*Y) + (L123c3*X)
        q1 = atan2(S1,C1)
        q4 = Yaw - q1 - q3
        q2 = Z+self.h4-self.h3-self.h1

        self.FK(q1=q1, q2=q2, q3=q3, q4=q4, init=1)

        return q1,q2,q3,q4

    def FK(self,q1,q2,q3,q4,init = 0):
        C1 = cos(q1)
        S1 = sin(q1)
        C13 = cos(q1+q3)
        S13 = sin(q1+q3)
        C134 = cos(q1 + q3 + q4)
        S134 = sin(q1 + q3 + q4)

        H1 =   [[C1,    -S1,    0,  0],
                [S1,    C1,     0,  0],
                [0,     0,      1,  self.h1],
                [0,     0,      0,  1]]

        H2 =   [[C1,    -S1,    0,  self.L1*C1],
                [S1,    C1,     0,  self.L1*S1],
                [0,     0,      1,  self.h1+q2],
                [0,     0,      0,  1]]
        
        H3 =   [[C13,    -S13,      0,      (self.L1+self.L2)*C1],
                [S13,    C13,       0,      (self.L1+self.L2)*S1],
                [0,     0,          1,       self.h1+self.h3+q2],
                [0,     0,          0,      1]]

        H4 =   [[C134,    -S134,    0,      (self.L3*C13)+((self.L1+self.L2)*C1)],
                [S134,    C134,     0,      (self.L3*S13)+((self.L1+self.L2)*S1)],
                [0,     0,      1,          self.h1+self.h3+q2],
                [0,     0,      0,          1]]
        
        He =   [[C134,    -S134,    0,      (self.L3*C13)+((self.L1+self.L2)*C1)],
                [S134,    C134,     0,      (self.L3*S13)+((self.L1+self.L2)*S1)],
                [0,     0,      1,          self.h1+self.h3-self.h4+q2],
                [0,     0,      0,          1]]

        
        if init == 0:
            self.H1 = H1
            self.H2 = H2
            self.H2 = H3
            self.H4 = H4
            self.He = He

            self.q1 = q1
            self.q2 = q2
            self.q3 = q3
            self.q4 = q4

            self.pose_X = He[0][3]
            self.pose_Y = He[1][3]
            self.pose_Z = He[2][3]
            self.pose_Yaw = q1 + q3 + q4

            self.C1 = C1
            self.C13 = C13
            self.C134 = C134
            self.C3 = cos(q3)
            self.S1 = S1
            self.S13 = S13
            self.S134 = S134
            self.S3 = sin(q3)

        self.Jacobian(init = 0)
        return H1, H2, H3, H4, He
        
    def Jacobian(self,init = 0):
        Je = [  [                                           0,    0,                0,      0],
                [                                           0,    0,                0,      0],
                [                                           1,    0,                1,      1],
                [(-self.L3*self.S13) - (self.L1*self.S1) - (self.L2*self.S1),    0,    (-self.L3*self.S13),     0],
                [ (self.L3*self.C13) + (self.L1*self.C1) + (self.L2*self.C1),    0,    (self.L3*self.C13),      0],
                [                                           0,    1,                0,      0]]
        
        if int(self.q3*1000) not in range(-210,210):        
            Je_inv = [  [0,                                    self.C13/(self.S3*(self.L12)),                                    self.S13/(self.S3*self.L12), 0],
                        [0,                                                                   0,                                                                   0, 1],
                        [0, -((self.L3*self.C13) + (self.L1*self.C1) + (self.L2*self.C1)) / (self.L3*self.S3*self.L12), -((self.L3*self.S13) + (self.L1*self.S1) + (self.L2*self.S1))/(self.L3*self.S3*self.L12), 0],
                        [1,                                                self.C1/(self.L3*self.S3),                                                self.S1/(self.L3*self.S3), 0]]
        else:
            Je_inv = None

        if init == 0:
            self.Je = Je
            self.Je_star = Je[2:5]
            self.Je_inv = Je_inv
        return Je,Je[2:5],Je_inv

    def IVK(self):
        return 0
    
    def IAK(self):
        return 0

    def Circle_Traj(self, Coor, Chessboard_Theta=0):
        r,theta = self.Coorchess_to_point[Coor]
        # X = r*cos(theta) + 0.745
        # Y = r*sin(theta) 
        theta_now = (theta + Chessboard_Theta)%(pi * 2)
        list_of_theta = np.arange(theta_now, pi*2, 0.02).tolist() + np.arange(0, theta_now, 0.02).tolist()
        # list_of_theta = [round(num, 3) for num in list_of_pose]

        list_of_pose = [(round((r*cos(theta_loop))+0.425, 3),round(r*sin(theta_loop), 3)) for theta_loop in list_of_theta]
        return list_of_pose
    def Cubic_Tarj(self, Initial, Goal):
        Coff1 = 0
        Coff2 = 0
        Coff3 = 0
        Coff4 = 0
        return Coff1,Coff2,Coff3,Coff4

    def Quintic_Tarj(self):

        return Coff1,Coff2,Coff3,Coff4,Coff5,Coff6


Robot = findchessbot()
print(Robot.IK(0.698245,0.,0,0.))
# print(Robot.FK(0.,0.,0.,0.))
# print(Robot.He)
# print(Robot.Jacobian(S13=0, C13=1, S1=0, C1=1)[2:5])
# Robot.FK(0.,0.,0.21,0.)
# print(Robot.Je_inv)
# list_of_pose = (Robot.Circle_Traj("A8"))
# list_of_pose = [[0.475, 0.05], [0.476, 0.051], [0.477, 0.052], [0.478, 0.053], [0.479, 0.054], [0.48, 0.055], [0.481, 0.056], [0.482, 0.057], [0.483, 0.058], [0.484, 0.059], [0.485, 0.06], [0.486, 0.061], [0.487, 0.062], [0.488, 0.063], [0.489, 0.064], [0.49, 0.065], [0.491, 0.066], [0.492, 0.067], [0.493, 0.068], [0.494, 0.069], [0.495, 0.07], [0.496, 0.071], [0.497, 0.072], [0.498, 0.073], [0.499, 0.074], [0.5, 0.075], [0.501, 0.076], [0.502, 0.077], [0.503, 0.078], [0.504, 0.079], [0.505, 0.08], [0.506, 0.081], [0.507, 0.082], [0.508, 0.083], [0.509, 0.084], [0.51, 0.085], [0.511, 0.086], [0.512, 0.087], [0.513, 0.088], [0.514, 0.089], [0.515, 0.09], [0.516, 0.091], [0.517, 0.092], [0.518, 0.093], [0.519, 0.094], [0.52, 0.095], [0.521, 0.096], [0.522, 0.097], [0.523, 0.098], [0.524, 0.099], [0.525, 0.1], [0.526, 0.101], [0.527, 0.102], [0.528, 0.103], [0.529, 0.104], [0.53, 0.105], [0.531, 0.106], [0.532, 0.107], [0.533, 0.108], [0.534, 0.109], [0.535, 0.11], [0.536, 0.111], [0.537, 0.112], [0.538, 0.113], [0.539, 0.114], [0.54, 0.115], [0.541, 0.116], [0.542, 0.117], [0.543, 0.118], [0.544, 0.119], [0.545, 0.12], [0.546, 0.121], [0.547, 0.122], [0.548, 0.123], [0.549, 0.124], [0.55, 0.125], [0.551, 0.126], [0.552, 0.127], [0.553, 0.128], [0.554, 0.129], [0.555, 0.13], [0.556, 0.131], [0.557, 0.132], [0.558, 0.133], [0.559, 0.134], [0.56, 0.135], [0.561, 0.136], [0.562, 0.137], [0.563, 0.138], [0.564, 0.139], [0.565, 0.14], [0.566, 0.141], [0.567, 0.142], [0.568, 0.143], [0.569, 0.144], [0.57, 0.145], [0.571, 0.146], [0.572, 0.147], [0.573, 0.148], [0.574, 0.149], [0.575, 0.15], [0.576, 0.151], [0.577, 0.152], [0.578, 0.153], [0.579, 0.154], [0.58, 0.155], [0.581, 0.156], [0.582, 0.157], [0.583, 0.158], [0.584, 0.159], [0.585, 0.16], [0.586, 0.161], [0.587, 0.162], [0.588, 0.163], [0.589, 0.164], [0.59, 0.165], [0.591, 0.166], [0.592, 0.167], [0.593, 0.168], [0.594, 0.169], [0.595, 0.17], [0.596, 0.171], [0.597, 0.172], [0.598, 0.173], [0.599, 0.174], [0.6, 0.175], [0.601, 0.176], [0.602, 0.177], [0.603, 0.178], [0.604, 0.179], [0.605, 0.18], [0.606, 0.181], [0.607, 0.182], [0.608, 0.183], [0.609, 0.184], [0.61, 0.185], [0.611, 0.186], [0.612, 0.187], [0.613, 0.188], [0.614, 0.189], [0.615, 0.19], [0.616, 0.191], [0.617, 0.192], [0.618, 0.193], [0.619, 0.194], [0.62, 0.195], [0.621, 0.196], [0.622, 0.197], [0.623, 0.198], [0.624, 0.199], [0.625, 0.2], [0.625, 0.2], [0.624, 0.199], [0.623, 0.198], [0.622, 0.197], [0.621, 0.196], [0.62, 0.195], [0.619, 0.194], [0.618, 0.193], [0.617, 0.192], [0.616, 0.191], [0.615, 0.19], [0.614, 0.189], [0.613, 0.188], [0.612, 0.187], [0.611, 0.186], [0.61, 0.185], [0.609, 0.184], [0.608, 0.183], [0.607, 0.182], [0.606, 0.181], [0.605, 0.18], [0.604, 0.179], [0.603, 0.178], [0.602, 0.177], [0.601, 0.176], [0.6, 0.175], [0.599, 0.174], [0.598, 0.173], [0.597, 0.172], [0.596, 0.171], [0.595, 0.17], [0.594, 0.169], [0.593, 0.168], [0.592, 0.167], [0.591, 0.166], [0.59, 0.165], [0.589, 0.164], [0.588, 0.163], [0.587, 0.162], [0.586, 0.161], [0.585, 0.16], [0.584, 0.159], [0.583, 0.158], [0.582, 0.157], [0.581, 0.156], [0.58, 0.155], [0.579, 0.154], [0.578, 0.153], [0.577, 0.152], [0.576, 0.151], [0.575, 0.15], [0.574, 0.149], [0.573, 0.148], [0.572, 0.147], [0.571, 0.146], [0.57, 0.145], [0.569, 0.144], [0.568, 0.143], [0.567, 0.142], [0.566, 0.141], [0.565, 0.14], [0.564, 0.139], [0.563, 0.138], [0.562, 0.137], [0.561, 0.136], [0.56, 0.135], [0.559, 0.134], [0.558, 0.133], [0.557, 0.132], [0.556, 0.131], [0.555, 0.13], [0.554, 0.129], [0.553, 0.128], [0.552, 0.127], [0.551, 0.126], [0.55, 0.125], [0.549, 0.124], [0.548, 0.123], [0.547, 0.122], [0.546, 0.121], [0.545, 0.12], [0.544, 0.119], [0.543, 0.118], [0.542, 0.117], [0.541, 0.116], [0.54, 0.115], [0.539, 0.114], [0.538, 0.113], [0.537, 0.112], [0.536, 0.111], [0.535, 0.11], [0.534, 0.109], [0.533, 0.108], [0.532, 0.107], [0.531, 0.106], [0.53, 0.105], [0.529, 0.104], [0.528, 0.103], [0.527, 0.102], [0.526, 0.101], [0.525, 0.1], [0.524, 0.099], [0.523, 0.098], [0.522, 0.097], [0.521, 0.096], [0.52, 0.095], [0.519, 0.094], [0.518, 0.093], [0.517, 0.092], [0.516, 0.091], [0.515, 0.09], [0.514, 0.089], [0.513, 0.088], [0.512, 0.087], [0.511, 0.086], [0.51, 0.085], [0.509, 0.084], [0.508, 0.083], [0.507, 0.082], [0.506, 0.081], [0.505, 0.08], [0.504, 0.079], [0.503, 0.078], [0.502, 0.077], [0.501, 0.076], [0.5, 0.075], [0.499, 0.074], [0.498, 0.073], [0.497, 0.072], [0.496, 0.071], [0.495, 0.07], [0.494, 0.069], [0.493, 0.068], [0.492, 0.067], [0.491, 0.066], [0.49, 0.065], [0.489, 0.064], [0.488, 0.063], [0.487, 0.062], [0.486, 0.061], [0.485, 0.06], [0.484, 0.059], [0.483, 0.058], [0.482, 0.057], [0.481, 0.056], [0.48, 0.055], [0.479, 0.054], [0.478, 0.053], [0.477, 0.052], [0.476, 0.051], [0.475, 0.05]]
# pose = []
# for i in list_of_pose:
#     q1,q2,q3,q4 = Robot.IK(X=i[0], Y=i[1], Z=0, Yaw=0)
#     pose.append([round(q1,3),round(q3,3)])
# print(pose)
# print(len(pose))