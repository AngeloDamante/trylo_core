#!/usr/bin/env python
"""Camera Node to generate image from camera Pi"""

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# create a webcam
webcam = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L)

class Camera(Node):
    def __init__(self, fps=30):
        super().__init__("n_camera")
        self.get_logger().info(f'[ CAMERA NODE ]: init {self.get_name()}')
        self.publisher = self.create_publisher(Image, "/vision/camera", 5)
        self.bridge = CvBridge()
        self.fps = fps
        self.timer = self.create_timer(1/self.fps, self.callback_imager)
        
    def callback_imager(self):
        flag, img = webcam.read()
        if flag:
            # self.get_logger().info('[ CAMERA NODE ]: image captured')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
            # self.get_logger().info('[ CAMERA NODE ]: image published')
            self.publisher.publish(msg)
        else:
            self.get_logger().info("Error")
            
    
def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    rclpy.spin(node)
    rclpy.shutdown()
    webcam.release()

if __name__ == "__main__":
    main()