import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import can

import transforms3d.euler
import math


class Receiver(Node):
    def __init__(self):
        super().__init__('receiver_node')
        self.subscription_1 = self.create_subscription(Twist, '/piracer/cmd_vel', self.callback_1, 10)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(Odometry, '/piracer/odom', self.callback_2, 10)
        self.subscription_2  # prevent unused variable warning

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')


    def callback_1(self, cmd_vel):
        steering = cmd_vel.angular.z
        throttle = cmd_vel.linear.x

        if steering < -1.0:
            steering = -1.0
        elif steering > 1.0:
            steering = 1.0

        if throttle < -1.0:
            throttle = -1.0
        elif throttle > 1.0:
            throttle = 1.0

        msg = can.Message(arbitration_id=0, data=[(steering > 0), int(abs(steering)), int((abs(steering) % 1) * 100), 0])
        self.bus.send(msg)

        msg = can.Message(arbitration_id=1, data=[(throttle < 0), int(abs(throttle)), int((abs(throttle) % 1) * 100), 0])
        self.bus.send(msg)

        # self.get_logger().info('')

    
    def callback_2(self, odom):
        x_pos = odom.pose.pose.position.x
        y_pos = odom.pose.pose.position.y

        orientation_list = [odom.pose.pose.orientation.w,
                            odom.pose.pose.orientation.x, 
                            odom.pose.pose.orientation.y, 
                            odom.pose.pose.orientation.z]
        roll, pitch, yaw = transforms3d.euler.quat2euler(orientation_list)
        yaw_degrees = math.degrees(yaw)

        msg = can.Message(arbitration_id=2, data=[(x_pos < 0), int(abs(x_pos)), int((abs(x_pos) % 1) * 100), 0])
        self.bus.send(msg)

        msg = can.Message(arbitration_id=3, data=[(y_pos < 0), int(abs(y_pos)), int((abs(y_pos) % 1) * 100), 0])
        self.bus.send(msg)

        msg = can.Message(arbitration_id=4, data=[(yaw_degrees < 0), int(abs(yaw_degrees)), 0, 0])
        self.bus.send(msg)

        # self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)

    receiver = Receiver()

    rclpy.spin(receiver)

    receiver.destroy_node()
    rclpy.shutdown()
