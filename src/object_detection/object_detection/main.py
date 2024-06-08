import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Classification2D, ObjectHypothesis
import message_filters
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

depthQue = [];
rgbQue = [];

class cameraSubscribe(Node):
    def __init__(self, signDetect):
        super().__init__('camera_sub_node');
        self.signDetect = signDetect;
        
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        self.ts = message_filters.TimeSynchronizer([self.rgb_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.callback)
        
        self.bridge = CvBridge();
        
    def callback(self, rgb_msg, depth_msg):
        self.get_logger().info(f'Image Received');
        
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8');
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1"); 
        
        rgbQue.append(rgb_image);
        depthQue.append(depth_image);

        self.signDetect.signInfo(rgbQue);

class signDetect:
    def __init__(self, model_path, signPublish, visualization):
        self.signPublish = signPublish;
        self.visualization = visualization;
        self.model = YOLO(model_path);
        self.resultQue = [];

    def signInfo(self, rgbQue):
        rgb_image = rgbQue.pop(0);
        results = self.model(rgb_image);

        for result in results:
            if len(result) > 0:
                self.resultQue.append(result);
                annotated_img = result.plot(); 
                self.visualization.pub_visual(annotated_img);
            else:
                pass;

        self.depthInfo(depthQue);


    def depthInfo(self, depthQue):
        depth_image = depthQue.pop(0);

        sign_id = "";
        dist = -1.0;
        conf = 0.0;

        while(len(self.resultQue) > 0):
            tmp = self.resultQue.pop(0);
            
            for tp in tmp:
                box = tp.boxes.xyxy;

                x = int((box[0][0] + box[0][2]) / 2);
                y = int((box[0][1] + box[0][3]) / 2);
                
                if(dist == -1.0 or depth_image[y, x].item() < dist):
                    conf = tp.boxes.conf;
                    dist = depth_image[y, x].item();
                    sign_id = self.model.names[int(tp.boxes.cls)];
        
        if(dist != -1.0 and conf >= 0.8):
            print(sign_id, dist);
            self.signPublish.pub_sign(sign_id, dist);
        else:   
            sign_id = "None";
            self.signPublish.pub_sign(sign_id, dist);

class signPublish(Node):
    def __init__(self):
        super().__init__("sign_pub_node");
        self.publisher_ = self.create_publisher(Classification2D, "/perception/sign", 10);
    
    def pub_sign(self, sign_id, score):
        msg = Classification2D();
        msg.header.stamp = self.get_clock().now().to_msg();
        msg.header.frame_id = "signs";

        classify = ObjectHypothesis();
        classify.id = sign_id;
        classify.score = score * 100;

        msg.results.append(classify);

        self.publisher_.publish(msg);

class visualization(Node):
    def __init__(self):
        super().__init__("visual_pub_node");
        self.publisher_ = self.create_publisher(Image, "/perception/visual", 10);
        self.bridge = CvBridge();
        
    def pub_visual(self, image):
    	self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(image), "bgr8"));

def main(args=None):
    rclpy.init(args=args);

    model_path = "src/object_detection/object_detection/models/gazebo/bestn.pt";

    sign_pub = signPublish();
    visual = visualization();
    sign_detect = signDetect(model_path, sign_pub, visual);
    node = cameraSubscribe(sign_detect);

    executor = MultiThreadedExecutor();
    executor.add_node(node);

    try:
        executor.spin();
    finally:
        node.destroy_node();
        rclpy.shutdown();

if __name__ == '__main__':
    main();
