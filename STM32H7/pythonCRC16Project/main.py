import serial
from CRC16 import CRC16

q1_theta = -8197
q2_theta = 286
q3_theta = 4562
q4_theta = 360

x_theta = 12
y_theta = 23
z_theta = 38
yaw_theta = 45


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


if __name__ == '__main__':
    crc16 = CRC16()
    data_joint_jog_q1 = [0x41, high_byte(convert_to_signed_14bit_data(q1_theta)),
                         low_byte(convert_to_signed_14bit_data(q1_theta))]
    frame_data_joint_jog_q1 = data_joint_jog_q1 + crc16.calculate(data_joint_jog_q1)
    print("Joint Jog Mode Calculation")
    print(frame_data_joint_jog_q1)
    print(dec_to_hex_in_list(frame_data_joint_jog_q1))
    print(serial.to_bytes(frame_data_joint_jog_q1))
