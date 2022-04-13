#!/usr/bin/env python3
from math import *
import numpy as np

class findchessbot():
    def __init__(self):
        self.L1 = 0.013245
        self.L2 = 0.370
        self.L3 = 0.315
        self.L12 = self.L1 + self.L2
        self.h1 = 0.125 
        self.h3 = 0.065
        self.h4 = 0.190

        self.Coorchess_to_point = {  #Theta, R
                'A1' : [0.247, 2.356],  'A2' : [0.215, 2.191],  'A3' : [0.19, 1.976],   'A4' : [0.177, 1.713],
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
        return q1,q2,q3,q4

    def FK(self,q1,q2,q3,q4):
        C1 = cos(q1)
        S1 = sin(q1)
        C13 = cos(q1+q3)
        S13 = sin(q1+q3)
        C134 = cos(q1 + q3 + q4)
        S134 = sin(q1 + q3 + q4)
        X = (self.L3*C13)+((self.L1+self.L2)*C1)
        Y = (self.L3*S13)+((self.L1+self.L2)*S1)
        Z = self.h1+self.h3-self.h4+q2
        Yaw = 0
        return X, Y, Z , Yaw

class Quintic_Traj():
    def __init__(self) -> None:
        pass

    def Traj_Gen(   self,
                    T,
                    Start_pos,
                    Final_pos,
                    Start_velocity=0,
                    Final_velocity=0,
                    Start_acceleration=0,
                    Final_acceleration=0  ):

        C0 = Start_pos
        C1 = Start_velocity
        C2 = Start_acceleration/2.0



        A = Final_pos - (Start_pos + (Start_velocity*T) + (Start_acceleration*T*T/2))
        B = Final_velocity - (Start_velocity + (Start_acceleration*T))
        C = Final_acceleration - Start_acceleration

        T2 = T*T
        T3 = T*T*T
        T4 = T*T*T*T
        T5 = T*T*T*T*T

        C3 = (10.0*A/T3) - (4.0*B/T2) + (C/(2.0*T))
        C4 = (-15.0*A/T4) + (7.0*B/T3) - (C/T2)
        C5 = (6.0*A/T5) - (3.0*B/T4) + (C/(2.0*T3))

        return C0,C1,C2,C3,C4,C5

    def Traj_Eval(self, C0, C1, C2, C3, C4, C5, t):
        t2 = t*t
        t3 = t*t*t
        t4 = t*t*t*t
        t5 = t*t*t*t*t
    
        S = C0 + (C1*t) + (C2*t2) + (C3*t3) + (C4*t4) + (C5*t5)
        V = C1 + (2.0*C2*t) + (3.0*C3*t2) + (4.0*C4*t3) + (5.0*C5*t4)
        A = (2.0*C2) + (6.0*C3*t) + (12.0*C4*t2) + (20.0*C5*t3)  

        return S, V, A

    def Jointspace_Traj(self, x1, y1, x2, y2):
        q1_Current, _, q3_Current, _ = findchessbot().IK(x1, y1, 0, 0)
        q1_Goal, _, q3_Goal, _ = findchessbot().IK(x2, y2, 0, 0)
        
        return (round(q1_Current,3)), (round(q3_Current,3)), (round(q1_Goal,3)), (round(q3_Goal,3))

    def Polar_to_Task(self,r, theta):
        return r*cos(theta), r*sin(theta)

A = Quintic_Traj()
B = findchessbot()
# x1,y1,_,_ = B.FK(0,0,0,0)
# x2,y2,_,_ = B.FK(-pi/2,0,pi/3,0)
# print(x1,y1,x2,y2)
# print(A.Jointspace_Traj(x1,y1,x2,y2))

omega = 2 * 0.10472
t = 10
r1, theta1 = B.Coorchess_to_point["H1"]
r2, theta2 = B.Coorchess_to_point["H4"]


x1, y1 = A.Polar_to_Task(r1, theta1)
x2, y2 = A.Polar_to_Task(r2, (theta2+(omega*t)))
print(x1,y1,x2,y2)
q1_Current, q3_Current,q1_Goal, q3_Goal = A.Jointspace_Traj(x1,y1,x2,y2)
print(A.Traj_Gen( T=t,
                Start_pos=q1_Current,
                Final_pos=q1_Goal))
print(A.Traj_Gen( T=t,
                Start_pos=q3_Current,
                Final_pos=q3_Goal))

