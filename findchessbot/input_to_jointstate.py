import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray, Header
from sensor_msgs.msg import JointState


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray,'/findchessbot/Joint_Pose',self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

    def pose_to_jointstate(self,inputy):
        return [inputy.data[0],inputy.data[1],inputy.data[2],inputy.data[3]]

    def listener_callback(self, submsg):
        # print(self.pose_to_jointstate(submsg))
        pubmsg = JointState()
        pubmsg.header = Header()
        pubmsg.header.stamp = MinimalSubscriber.get_clock(self).now().to_msg()
        pubmsg.position = self.pose_to_jointstate(submsg)
        pubmsg.name = ["joint_1","joint_2","joint_3","joint_4"]
        pubmsg.velocity = []
        pubmsg.effort = []
        self.publisher_.publish(pubmsg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
