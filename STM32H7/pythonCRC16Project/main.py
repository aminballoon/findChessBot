import serial
from CRC16 import CRC16
import time, threading
import keyboard




q1_theta = 0
q2_theta = 0
q3_theta = 0
q4_theta = 0

x_theta = 0
y_theta = 0
z_theta = 0
yaw_theta = 0


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

def foo():
    # print(time.ctime())
    data_joint_jog_q1 = [0x41, high_byte((q1_theta)),low_byte((q1_theta))]
    frame_data_joint_jog_q1 = data_joint_jog_q1 + crc16.calculate(data_joint_jog_q1)

    data_joint_jog_q2 = [0x42, high_byte((q2_theta)),low_byte((q2_theta))]
    frame_data_joint_jog_q2 = data_joint_jog_q2 + crc16.calculate(data_joint_jog_q2)

    data_joint_jog_q3 = [0x43, high_byte((q3_theta)),low_byte((q3_theta))]
    frame_data_joint_jog_q3 = data_joint_jog_q3 + crc16.calculate(data_joint_jog_q3)

    data_joint_jog_q4 = [0x44, high_byte((q4_theta)),low_byte((q4_theta))]
    frame_data_joint_jog_q4 = data_joint_jog_q4 + crc16.calculate(data_joint_jog_q4)

    ser.write(bytearray(frame_data_joint_jog_q1))
    time.sleep(0.1)
    ser.write(bytearray(frame_data_joint_jog_q2))
    time.sleep(0.1)
    ser.write(bytearray(frame_data_joint_jog_q3))
    time.sleep(0.1)
    ser.write(bytearray(frame_data_joint_jog_q4))

    threading.Timer(0.5, foo).start()

if __name__ == '__main__':
    ser = serial.Serial('COM8', 115200, timeout=1)
    crc16 = CRC16()
    # data_joint_jog_q1 = [0x41, high_byte((q1_theta)),low_byte((q1_theta))]
    # data_joint_jog_q1 = [0x42,*bytesy(q2_theta)]
    # frame_data_joint_jog_q1 = data_joint_jog_q1 + crc16.calculate(data_joint_jog_q1)
    print("Joint Jog Mode Calculation")
    # print(frame_data_joint_jog_q1)
    # print(dec_to_hex_in_list(frame_data_joint_jog_q1))
    # print(serial.to_bytes(frame_data_joint_jog_q1))
    # foo()

    try:
        while(1):
            q1_theta = 0
            q2_theta = 0
            q3_theta = 0
            q4_theta = 0
            if keyboard.is_pressed('q'):
                q1_theta = 5

            if keyboard.is_pressed('a'):
                q1_theta = -5

            if keyboard.is_pressed('w'):
                q2_theta = 5

            if keyboard.is_pressed('s'):
                q2_theta = -5

            if keyboard.is_pressed('e'):
                q3_theta = 5

            if keyboard.is_pressed('d'):
                q3_theta = -5

            if keyboard.is_pressed('r'):
                q4_theta = 5

            if keyboard.is_pressed('f'):
                q4_theta = -5

            frame = [0x61,low_byte(q1_theta),low_byte(q2_theta),low_byte(q3_theta),low_byte(q4_theta)]
            data = frame + crc16.calculate(frame)
            ser.write(data)
            time.sleep(0.1)



            # while(1):
            #     x_theta = 0
            #     y_theta = 0
            #     z_theta = 0
            #     yaw_theta = 0
            #     if keyboard.is_pressed('w'):
            #         x_theta = 120

            #     if keyboard.is_pressed('s'):
            #         x_theta = -120

            #     if keyboard.is_pressed('a'):
            #         y_theta = 120

            #     if keyboard.is_pressed('d'):
            #         y_theta = -120


            #     frame = [0x71,low_byte(x_theta),low_byte(y_theta),low_byte(z_theta),low_byte(yaw_theta)]
            #     data = frame + crc16.calculate(frame)
            #     ser.write(data)
            #     time.sleep(0.1)

    except KeyboardInterrupt:
        pass
            
    