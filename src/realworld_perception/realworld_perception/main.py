import rclpy    
from rclpy.node import Node    
from vision_msgs.msg import Classification2D, ObjectHypothesis    
import cv2    
from cv_bridge import CvBridge
import numpy as np    
from ultralytics import YOLO   
import pyrealsense2 as rs

class Camera:
    def __init__(self, detect):
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 320, 240, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline = rs.pipeline()
        self.pipeline.start(self.config)

        self.detect = detect

    def processImage(self, detect):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not depth_frame or not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())[85:-90, 100:-135]  # Trim edge [v, h]

        detect.detectSign(color_image)

    def __del__(self):
        self.pipeline.stop()

class signDetection:
    def __init__(self, model_path, publisher):
        self.model = YOLO(model_path)
        self.publisher = publisher

    def detectSign(self, cv_image):
        detections = self.model(cv_image)

        for result in detections:
            if len(result) > 0:
                sign_id = self.model.names[int(result[0].boxes.cls)]
                score = 0.7
                self.pub_sign(sign_id, score)
            self.pub_sign("none", 0.0)

    def pub_sign(self, sign_id, score):
        self.publisher.sign_info(sign_id, score)

class signPublisher(Node):
    def __init__(self):
        super().__init__("sign_publisher")
        self.publisher_ = self.create_publisher(Classification2D, "/perception/sign", 10)
        self.get_logger().info("Publish Start")

    def sign_info(self, sign_id, score):
        msg = Classification2D()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "signs"

        classify = ObjectHypothesis()
        classify.id = sign_id
        classify.score = score

        msg.results.append(classify)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ID="{sign_id}"')

def main(args=None):
    rclpy.init(args=args)

    model_path = "src/simulation_perception/simulation_perception/models/real/bestn.pt"
    
    sign_publisher = signPublisher()
    sign_detect = signDetection(model_path, sign_publisher)
    camera = Camera(sign_detect)

    try:
        while(1):
            camera.processImage(sign_detect)
    except KeyboardInterrupt:
        pass
    finally:
        camera.__del_()
        sign_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

