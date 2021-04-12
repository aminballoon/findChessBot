import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState


class MinimalPublisher(Node):

    def __init__(self,t):
        self.t = t
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointState, 'topic', 10)
        timer_period = self.t   # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = JointState()
        # msg.data = 'Hello World: %d' % self.i
        msg.position = [0.1,0.02,0.5,-1.]
        msg.name = ["joint_1,joint_2,joint_3,joint_4"]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
 


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(t=0.5)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
