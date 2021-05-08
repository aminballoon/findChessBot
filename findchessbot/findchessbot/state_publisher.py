from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion, PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32
import numpy as np





class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.subscription = self.create_subscription(Float32,'/findchessbot/chessboard_rpm',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        
        degree = pi / 180.0

        self.joint_state = JointState()
        
        self.chessboard_orientation = 0.

        self.rate = 0.001
        self.timer = self.create_timer(self.rate, self.timer_callback)
        
        self.circle_angle = pi*2.
        


    def timer_callback(self):
        # update joint_state
        q2 = -0.135
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_chess']
        self.joint_state.position = [0., q2, 0., 0., self.chessboard_orientation ]

        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
                

    def listener_callback(self, msg):
        self.chessboard_orientation = msg.data

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    state_publisher = StatePublisher()
    rclpy.spin(state_publisher)

if __name__ == '__main__':
    main()
