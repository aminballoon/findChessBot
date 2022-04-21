import serial
from CRC16 import CRC16
import time, threading
import keyboard
import random
def low_byte(_input):
    return int(int(_input) & 0xFF)


def high_byte(_input):
    return int((int(_input) >> 8) & 0xFF)


def convert_to_signed_14bit_data(_input):
    if -8191 <= _input <= 8191:
        if _input < 0:
            return 0x2000 | abs(_input)
        else:
            return _input
    elif _input < -8191:
        return 0x2000 | 8191
    elif _input > 8191:
        return 8191
    else:
        return 0


def convert_to_unsigned_8bit_data_lim(_input, _l_lim, _u_lim):
    if _input < _l_lim:
        return int(_l_lim)
    elif _input > _u_lim:
        return int(_u_lim)
    else:
        return int(_input)


def dec_to_hex_in_list(_input):
    result = []
    for x in _input:
        if len(str(hex(x)[2:].upper())) >= 2:
            result.append(str(hex(x)[2:].upper()))
        else:
            result.append('0' + str(hex(x)[2:].upper()))
    return result

def bytesy(integer):
        return divmod(integer, 0x100)

ch_sq = {
}
num = 0
for i in ['a','b','c','d','e','f','g','h']:
    for j in range(1,9):
        o = str(i+str(j))
        ch_sq.update({o: num})
        num += 1
# print(ch_sq['b1'])

if __name__ == '__main__':
    try:
        ser = serial.Serial('COM8', 115200, timeout=1)
        crc16 = CRC16()

        # frame1 = [0x86,0]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # x = random.randint(3, 6)

        # frame1 = [0x86,37,0]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # frame1 = [0x86,20,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # frame1 = [0x86,4,0]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # frame1 = [0x86,48,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,53,0]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # frame1 = [0x86,20,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # frame1 = [0x86,60,0]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
        # frame1 = [0x86,34,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

    
        # control_state = 41
        # frame1 = [0x87,control_state]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
 

        frame1 = [0x86,ch_sq['a1'],1]
        data1 = frame1 + crc16.calculate(frame1)
        ser.write(data1)
        time.sleep(0.1)

        # frame1 = [0x86,ch_sq['b2'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['c3'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['d4'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['e5'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['f6'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['g7'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        frame1 = [0x86,ch_sq['h8'],2 ]
        data1 = frame1 + crc16.calculate(frame1)
        ser.write(data1)
        time.sleep(0.1)

        # frame1 = [0x86,ch_sq['c4'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['d6'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['a1'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        control_state = 41
        frame1 = [0x87,control_state]
        data1 = frame1 + crc16.calculate(frame1)
        ser.write(data1)

        # num = 8
        # for i in range(num,num+8):
        #     frame1 = [0x86,i*3]
        #     data1 = frame1 + crc16.calculate(frame1)
        #     ser.write(data1)
        #     time.sleep(1)

        
        # # Gripper Close 
        # frame1 = [0x81,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.5)

        # # Gripper Open
        # frame1 = [0x81,0]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.5)

        # frame1 = [0x89,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.5)
        # frame1 = [0x89,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)

        


        # control_state = 0;
        # frame1 = [0x87,control_state]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)


        # control_state = 42; # Down
        # frame1 = [0x87,control_state]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(1)
        # frame1 = [0x89,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(1)
        # frame1 = [0x89,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)

        # control_state = 43; # Up
        # frame1 = [0x87,control_state]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(1)
        # frame1 = [0x89,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(1)
        # frame1 = [0x89,1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)


    except KeyboardInterrupt:
        pass
            
    