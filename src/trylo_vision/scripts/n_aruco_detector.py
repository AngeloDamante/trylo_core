#!/usr/bin/env python
"""Aruco Detector node to detect markers"""

import cv2
from cv2 import aruco
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from trylo_vision.msg import MarkersDetected, Marker, Pair

dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250) 

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('n_aruco_detector')
        self.get_logger().info(f'[ ARUCO NODE ]: init {self.get_name()}')
        self.subscription = self.create_subscription(Image, "/vision/camera", self.callback, 5)
        self.publisher = self.create_publisher(msg_type=MarkersDetected, topic="/vision/aruco", qos_profile=10)
        self.bridge = CvBridge()
    
    def callback(self, msg):
        markers_detected = MarkersDetected()
        self.get_logger().info('[ ARUCO NODE ]: received image')
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(frame, dictionary, parameters=aruco.DetectorParameters())
        if len(corners) > 0:
            self.get_logger().info('[ ARUCO NODE ]: aruco detected')
            self.get_logger().info(f'[ ARUCO NODE ]: ids = {ids}')
            self.get_logger().info(f'[ ARUCO NODE ]: corners = {corners}')
            _markers_detected = []
            _ids_detected = []
            _marker_detected = Marker()
            _corner = Pair()
            for i in range(len(ids)):    
                _corner.x = int(corners[i][0][0][0])
                _corner.y = int(corners[i][0][0][1])
                _marker_detected.top_left = copy.deepcopy(_corner)
                
                _corner.x = int(corners[i][0][1][0])
                _corner.y = int(corners[i][0][1][1])
                _marker_detected.top_right = copy.deepcopy(_corner)
                
                _corner.x = int(corners[i][0][2][0])
                _corner.y = int(corners[i][0][2][1])
                _marker_detected.bottom_right = copy.deepcopy(_corner)
                
                _corner.x = int(corners[i][0][3][0])
                _corner.y = int(corners[i][0][3][1])
                _marker_detected.bottom_left = copy.deepcopy(_corner)
                
                _ids_detected.append(int(ids[i][0]))
                _markers_detected.append(_marker_detected)
            markers_detected.ids = _ids_detected
            markers_detected.corners = _markers_detected
            self.publisher.publish(markers_detected)
        

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()