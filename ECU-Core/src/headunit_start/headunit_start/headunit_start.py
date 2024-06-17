import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8

import can


class HeadunitStart(Node):
    def __init__(self):
        super().__init__('headunit_start_node')
        self.publisher_ = self.create_publisher(Int8, '/headunit/start', 10)

        self.msg = Int8()
        self.msg.data = 0
        self.publisher_.publish(self.msg)
        # self.get_logger().info('')


def main(args=None):
    rclpy.init(args=args)
    headunit_start = HeadunitStart()

    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    try:
        while True:
            message = bus.recv()

            if message.arbitration_id == 5:
                headunit_start.msg.data = message.data[0]
                headunit_start.publisher_.publish(headunit_start.msg)

    except KeyboardInterrupt:
        bus.shutdown()

        headunit_start.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
