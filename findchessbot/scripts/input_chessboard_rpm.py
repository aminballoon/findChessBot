#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import pi,sin,cos
from std_msgs.msg import String,Float32
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self,rate):
        
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Float32, '/findchessbot/chessboard_rpm', 10)
        self.rate = rate
        timer_period = self.rate   # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.time = 0.
        self.circle_angle = pi*2.
    def timer_callback(self):
        
        msg = Float32()
        msg.data = (0.20944 * self.time) % self.circle_angle 
        # msg.data = 'Hello World: %d' % self.i
        # msg.position = [0.1,0.02,0.5,-1.]
        # msg.name = ["joint_1,joint_2,joint_3,joint_4"]
        self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)
        self.time += self.rate
 


def main(args=None):

    rclpy.init(args=args)

    rate = 1./50.
    minimal_publisher = MinimalPublisher(rate = rate)

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
