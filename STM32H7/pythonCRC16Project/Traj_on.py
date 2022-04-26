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
# print(ch_sq['b1'])

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
 

        # frame1 = [0x86,ch_sq['a1'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['d8'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['a8'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['b2'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['h1'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)
 
        # frame1 = [0x86,ch_sq['e1'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['h8'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['e5'],2]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        # frame1 = [0x86,ch_sq['h8'],1 ]
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

        # frame1 = [0x86,ch_sq['a1'],1]
        # data1 = frame1 + crc16.calculate(frame1)
        # ser.write(data1)
        # time.sleep(0.1)

        control_state = 41
        frame1 = [0x87,control_state]
        data1 = frame1 + crc16.calculate(frame1)
        ser.write(data1)


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
            

# ch_sq = {
# }
# num = 0
# for i in ['a','b','c','d','e','f','g','h']:
#     for j in range(1,9):
#         o = str(i+str(j))
#         ch_sq.update({o: num})
#         num += 1
# print(ch_sq)