import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import pyrealsense2 as rs
import numpy as np
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_pub_node')
        self.publisher1_ = self.create_publisher(Image, 'image_RGB', 10)    # Output1: RGB
        self.publisher2_ = self.create_publisher(Image, 'image_Depth', 10)  # Output2: Depth
        self.timer_ = self.create_timer(0.2, self.callback)  # [s]

        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)

        self.bridge = CvBridge()
        self.msg_RGB = Image()
        self.msg_Depth = Image()


    def callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())[85:-90, 100:-135]  # Trim edge [v, h]

        self.msg_RGB = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.msg_Depth = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")

        self.publisher1_.publish(self.msg_RGB)
        self.publisher2_.publish(self.msg_Depth)
        # self.get_logger().info('')


    def __del__(self):
        self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()