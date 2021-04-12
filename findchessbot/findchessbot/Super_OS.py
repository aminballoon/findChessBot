from math import sqrt ,sin ,cos ,atan2
class Robot():
    def __init__(self):
        self.L1 = 0.020
        self.L2 = 0.370
        self.L3 = 0.355
        self.L12 = 0.390
        self.H1 = 0.125 
        self.H3 = 0.065
        self.H4 = 0.190

    def IK(self,X,Y,Z,Yaw):
        C3 = ((X*X) + (Y*Y) - (pow(self.L12,2)) - (pow(self.L3,2)) ) / (2*self.L12*self.L3)
        print(C3)
        S3 = sqrt(1-pow(C3,2))
        q3 = atan2(S3,C3)
        L3s3 = self.L3*S3
        L123c3 = self.L12+(self.L3*C3)
        S1 = (-L3s3*X) + (L123c3*Y);
        C1 = (L3s3*Y) + (L123c3*X);
        q1 = atan2(S1,C1);
        q4 = Yaw - q1 - q3;
        q2 = Z+self.H4-self.H3-self.H1;

        return q1,q2,q3,q4

        
        

findchessbot = Robot()
print(findchessbot.IK(0.600,0.,0.120,0.))