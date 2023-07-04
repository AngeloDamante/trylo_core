#!/usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from trylo_vision.msg import MarkersDetected

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) 

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('n_aruco_detector')
        self.get_logger(f'[ ARUCO NODE ]: init {self.get_name()}')
        self.subscription = self.create_subscription(Image, "/vision/camera", self.callback, 5)
        self.publisher = self.create_publisher(msg_type=MarkersDetected, topic="/vision/aruco", qos_profile=10)
        self.bridge = CvBridge()
    
    def callback(self, msg):
        markers_detected = MarkersDetected()
        self.get_logger().debug('[ ARUCO NODE ]: received image')
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
        self.get_logger().debug('[ ARUCO NODE ]: detection aruco')
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=aruco.DetectorParameters())
        if len(corners) > 0:
            self.get_logger().debug('[ ARUCO NODE ]: aruco detected')
            self.get_logger().debug('[ ARUCO NODE ]: corners = {corners}')
            for i in range(len(ids)):                
                markers_detected.ids[i] = ids
                markers_detected.corners[i].top_left = corners[i][0]
                markers_detected.corners[i].top_right = corners[i][1]
                markers_detected.corners[i].bottom_right = corners[i][2]
                markers_detected.corners[i].bottom_left = corners[i][3]
            self.publisher.publish(markers_detected)
        

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()