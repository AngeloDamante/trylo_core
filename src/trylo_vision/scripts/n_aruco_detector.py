#!/usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) 

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('n_aruco_detector')
        self.get_logger(f'[ ARUCO NODE ]: init {self.get_name()}')
        self.subscription = self.create_subscription(Image, "/vision/camera", self.callback, 5)
        # self.publisher = self.crea # TODO
        self.bridge = CvBridge()
    
    def callback(self, msg):
        self.get_logger().debug('[ ARUCO NODE ]: received image')
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        self.get_logger().debug('[ ARUCO NODE ]: detection aruco')
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=aruco.DetectorParameters())
        if len(corners) > 0:
            self.get_logger().debug('[ ARUCO NODE ]: aruco detected')
            aruco.drawDetectedMarkers(img, corners)
        
        

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()