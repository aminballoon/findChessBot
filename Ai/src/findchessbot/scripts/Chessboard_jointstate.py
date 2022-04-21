#!/usr/bin/env python3
import rclpy
import sys
from rclpy.node import Node
from math import pi,sin,cos
from std_msgs.msg import String,Float32
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile

class Chessboard_Orientation_Pubilhser(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(depth=10)
        self.publisher_CB_Orientation = self.create_publisher(Float32, '/findchessbot/chessboard_orientation', qos_profile)
        self.publisher_CB_JointState = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.rate = 0.02
        self.timer = self.create_timer(self.rate, self.timer_callback)
        self.time = 0.
        self.circle_angle = pi*2.0
        self.joint_state = JointState()

    def timer_callback(self):
        msg = Float32()
        chessboard_orientation = (0.20944 * self.time) % self.circle_angle 
        msg.data = chessboard_orientation
        self.publisher_CB_Orientation.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)
        self.time += self.rate
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['joint_chess']
        self.joint_state.position = [chessboard_orientation]
        self.joint_state.velocity = [self.circle_angle]
        # send the joint state and transform
        self.publisher_CB_JointState .publish(self.joint_state)

 

def main(args=None):
    try:
        rclpy.init(args=args)
        CBP = Chessboard_Orientation_Pubilhser()
        rclpy.spin(CBP)
    except KeyboardInterrupt:
        # quit
        CBP.destroy_node()
        rclpy.shutdown()
        sys.exit()


if __name__ == '__main__':
    main()
