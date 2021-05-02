from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        degree = pi / 180.0
        loop_rate = self.create_rate(20)

        # robot state
        # tilt = 0.
        # tinc = degree
        # swivel = 0.
        # angle = 0.
        # height = 0.
        # hinc = 0.005

        # message declarations
        # transform = TransformStamped()
        # transform.header.frame_id = 'world'
        # transform.child_frame_id = 'base_link'
        joint_state = JointState()
        Hello = list(range(0, 314,2))
        isas = list(range(0, 314*2,1))
        i = 0
        state = 0
        j = 0
        k = len(isas)-1
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                q2 = -0.135
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4','joint_chess']
                joint_state.position = [Hello[i]/-100. , q2 , Hello[i]/100. , 0., isas[j]/100.]
                # joint_state.position = [0., q2, 0., 0., pi/4.]
                if state == 0:
                    i += 1
                else:
                    i -= 1

                if i == 150:
                    state = 1
                   
                elif i == 0:
                    state = 0
                
                if j == k:
                    j = 0
                else:
                    j += 1
                
                # update transform
                # (moving in a circle with radius=2)
                # transform.header.stamp = now.to_msg()
                # transform.transform.translation.x = 0.
                # transform.transform.translation.y = 0.
                # transform.transform.translation.z = 0.
                # transform.transform.rotation = \
                #     euler_to_quaternion(0., 0., 0.) # roll,pitch,yaw

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                # self.broadcaster.sendTransform(transform)

                # Create new robot state
                # tilt += tinc
                # if tilt < -0.5 or tilt > 0.0:
                #     tinc *= -1
                # height += hinc
                # if height > 0.2 or height < 0.0:
                #     hinc *= -1
                # swivel += degree
                # angle += degree/4

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()
