import serial

class Communication:
    def __init__(self, baud, port):
        self.mode = 0
        self.baud = baud
        self.port = port
    def bytesy(self,integer):
        return divmod(integer, 0x100)

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

    def mode_6(self,q1,q2,q3,q4): # q1 mode
        Joint_1 = q1 # rad
        if Joint_1 >= 0:
            buff_J1 = Joint_1
        else:
            buff_J1 = -Joint_1 + 32768

        buff = ([self.mode_to_start(6),self.bytesy(buff_J1)])
        return buff + self.checksum(buff)

    def mode_7(self,q2): # q1 mode
        buff = ([self.mode_to_start(7),q2])
        return buff + self.checksum(buff)

    def mode_8(self,q3): 
        Joint_3 = q3 # rad
        if Joint_3 >= 0:
            buff_J3 = Joint_3
        else:
            buff_J3 = -Joint_3 + 32768
        buff = ([self.mode_to_start(8),self.bytesy(buff_J3)])
        return buff + self.checksum(buff)

    def mode_9(self,q4): 
        Joint_4 = q4 # rad
        if Joint_4 >= 0:
            buff_J4 = Joint_4
        else:
            buff_J4 = -Joint_4 + 32768
        buff = ([self.mode_to_start(9),self.bytesy(buff_J4)])
        return buff + self.checksum(buff)



    def mode_10(self): # Set Home
        buff = ([self.mode_to_start(10)])
        return buff + self.checksum(buff)

    def mode_11(self): # Request 4 Joint State
        buff = ([self.mode_to_start(11)])
        return buff + self.checksum(buff)

    def mode_12(self): # Request Gripper State
        buff = ([self.mode_to_start(12)])
        return buff + self.checksum(buff)

    def mode_13(self): # Activate Gripper
        buff = ([self.mode_to_start(13)])
        return buff + self.checksum(buff)

    def mode_14(self): # Deactivate Gripper
        buff = ([self.mode_to_start(14)])
        return buff + self.checksum(buff)

    def mode_15(self):
        return 0




O = Communication(baud=512000,port= 'COM15')
O.set_mode(10)
# print(O.mode)
P = (O.mode_6(90,0,155,126))
