from math import sqrt ,sin ,cos ,atan2, pi
import numpy as np
class findchessbot():
    def __init__(self):
        self.L1 = 0.020
        self.L2 = 0.370
        self.L3 = 0.355
        self.L12 = 0.390
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

    def IK(self,X,Y,Z,Yaw):
        C3 = ((X*X) + (Y*Y) - (pow(self.L12,2)) - (pow(self.L3,2)) ) / (2*self.L12*self.L3)
        print(C3)
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

    def Coorchess_to_point(self, Initial, Goal):
        Dict = {"A": -3.5, "B": -2.5,"C": -1.5, "D": -0.5, 
                "H": -3.5, "G": -2.5,"F": -1.5,"E": -0.5,
                "1": -3.5, "2": -2.5,"3": -1.5, "4": -0.5, 
                "8": -3.5, "7": -2.5,"6": -1.5,"5": -0.5}

        Initial_Pose = [round(num, 3) for num in [(.05*Dict[Initial[0]])+.745, .05*Dict[Initial[1]]]]
        Goal_Pose = [round(num, 3) for num in [(.05*Dict[Goal[0]])+.745, .05*Dict[Goal[1]]]]
        
        return Initial_Pose,Goal_Pose

    def Cubic_Tarj(self, Initial, Goal):
        Coff1 = 0
        Coff2 = 0
        Coff3 = 0
        Coff4 = 0
        return Coff1,Coff2,Coff3,Coff4

    def Quintic_Tarj(self):

        return Coff1,Coff2,Coff3,Coff4,Coff5,Coff6


Robot = findchessbot()
# print(Robot.IK(0.600,0.,0.120,0.))
# print(Robot.FK(30,0.,0.,0.))
# print(Robot.He)
# print(Robot.Jacobian(S13=0, C13=1, S1=0, C1=1)[2:5])
# Robot.FK(0.,0.,0.21,0.)
# print(Robot.Je_inv)

print(Robot.Coorchess_to_point(Initial="A8", Goal="B6"))