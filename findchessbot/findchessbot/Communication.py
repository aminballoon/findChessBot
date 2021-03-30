import serial

class Communication:
    def __init__(self, baud, port):
        self.mode = 0
        self.baud = baud
        self.port = port
    def set_mode(self,mode_input):
        self.mode = mode_input
    def mode_to_start(self,num_mode):
        return (( 0b1010 << 4 ) & 0xF0 ) | num_mode
    def checksum(self,Data_Frame):
        return ([( ~(sum(Data_Frame)%256))%256])


    def mode_1(self,character):
        buff = ([self.mode_to_start(1),ord(character)])
        return buff + self.checksum(buff)

    def mode_2(self):
        buff = ([self.mode_to_start(2)])
        return buff + self.checksum(buff)

    def mode_3(self):
        buff = ([self.mode_to_start(3)])
        return buff + self.checksum(buff)

    def mode_4(self):
        return 0
    
    def mode_5(self):
        buff = ([self.mode_to_start(5)])
        return buff + self.checksum(buff)

    def mode_6(self,J1,J2,J3,J4): # 4 Joint State Command
        Joint_1 = J1 # rad
        Joint_2 = J2 # mm
        Joint_3 = J3 # rad
        Joint_4 = J4 # rad

        if Joint_1 >= 0:
            buff_J1 = Joint_1
        else:
            buff_J1 = -Joint_1 + 128

        if Joint_3 >= 0:
            buff_J2 = Joint_2
            buff_J3 = Joint_3
        else:
            buff_J2 = (Joint_2 << 1) + 1
            buff_J3 = -Joint_3
            
        if Joint_4 >= 0:
            buff_J4 = Joint_4
        else:
            buff_J4 = -Joint_4 + 128

        buff = ([self.mode_to_start(6),buff_J1,buff_J2,buff_J3,buff_J4])
        return buff + self.checksum(buff)

    def mode_7(self): 
        return 0

    def mode_8(self): # Set Home
        buff = ([self.mode_to_start(8)])
        return buff + self.checksum(buff) 

    def mode_9(self): # Request 4 Joint State
        buff = ([self.mode_to_start(9)])
        return buff + self.checksum(buff)

    def mode_10(self): # Request Gripper State
        buff = ([self.mode_to_start(10)])
        return buff + self.checksum(buff)

    def mode_11(self): # Activate Gripper
        buff = ([self.mode_to_start(11)])
        return buff + self.checksum(buff)

    def mode_12(self): # Deactivate Gripper
        buff = ([self.mode_to_start(12)])
        return buff + self.checksum(buff)

    def mode_13(self):
        return 0

    def mode_14(self):
        return 0

    def mode_15(self):
        return 0




O = Communication(baud=512000,port= 'COM15')
O.set_mode(10)
# print(O.mode)
P = (O.mode_6(90,0,155,126))
