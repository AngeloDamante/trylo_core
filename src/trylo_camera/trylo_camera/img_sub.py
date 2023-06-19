#!/usr/bin/env python

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ImageReader(Node):
    def __init__(self):
        super().__init__('img_sub')
        self.subscription = self.create_subscription(Image, "camera_data", self.callback, 5)
        self.bridge = CvBridge()
    
    def callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info(str(img.shape))

def main():
    rclpy.init()
    node = ImageReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()