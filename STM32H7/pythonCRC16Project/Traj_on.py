import serial
from CRC16 import CRC16
import time, threading
# import keyboard
import random

ch_sq = {
}
num = 0
for i in ['a','b','c','d','e','f','g','h']:
    for j in range(1,9):
        o = str(i+str(j))
        ch_sq.update({o: num})
        num += 1
print(ch_sq)
p = {   'a1':  0, 'a2':  1, 'a3':  2, 'a4':  3, 'a5':  4, 'a6':  5, 'a7':  6, 'a8':  7, 
        'b1':  8, 'b2':  9, 'b3': 10, 'b4': 11, 'b5': 12, 'b6': 13, 'b7': 14, 'b8': 15, 
        'c1': 16, 'c2': 17, 'c3': 18, 'c4': 19, 'c5': 20, 'c6': 21, 'c7': 22, 'c8': 23, 
        'd1': 24, 'd2': 25, 'd3': 26, 'd4': 27, 'd5': 28, 'd6': 29, 'd7': 30, 'd8': 31, 
        'e1': 32, 'e2': 33, 'e3': 34, 'e4': 35, 'e5': 36, 'e6': 37, 'e7': 38, 'e8': 39, 
        'f1': 40, 'f2': 41, 'f3': 42, 'f4': 43, 'f5': 44, 'f6': 45, 'f7': 46, 'f8': 47, 
        'g1': 48, 'g2': 49, 'g3': 50, 'g4': 51, 'g5': 52, 'g6': 53, 'g7': 54, 'g8': 55, 
        'h1': 56, 'h2': 57, 'h3': 58, 'h4': 59, 'h5': 60, 'h6': 61, 'h7': 62, 'h8': 63}

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
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

        frame1 = [0x86,ch_sq['d4'],2]
        data1 = frame1 + crc16.calculate(frame1)
        ser.write(data1)
        time.sleep(0.1)

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

        # frame1 = [0x86,ch_sq['h8'],2 ]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

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
            
    