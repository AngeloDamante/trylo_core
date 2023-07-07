#!/usr/bin/env python
"""Aruco Drawer node to draw detected markers"""

import cv2
from cv2 import aruco
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from trylo_vision.msg import MarkersDetected, Marker, Pair
from src.parameters import FPS


class ArucoDrawer(Node):
    def __init__(self, fps=FPS):
        super().__init__('n_aruco_drawer')
        self.get_logger().info(f'[ ARUCO DRAWER NODE ]: init {self.get_name()}')
        self.subscription_aruco = self.create_subscription(MarkersDetected, "/vision/aruco", self.cbk_get_corners, 5)
        self.subscription_image = self.create_subscription(Image, "/vision/camera", self.cbk_get_img, 5)
        self.publisher = self.create_publisher(Image, "/vision/aruco_cam", 10)

        # attributes
        self.bridge = CvBridge()
        self.markers_detected = MarkersDetected()
        
        self.image_camera = None
        self.image_markers_drawed = None

    def cbk_get_img(self, msg_img):
        # take image
        self.get_logger().info('[ ARUCO DRAWER NODE ]: received image')
        self.image_camera = self.bridge.imgmsg_to_cv2(msg_img, desired_encoding="passthrough")

        # publish
        if self.image_markers_drawed is None:
            msg = self.bridge.cv2_to_imgmsg(self.image_camera, encoding="passthrough")
            self.publisher.publish(msg)


    def cbk_get_corners(self, msg_marker):
        ids, corners = msg_marker.ids, msg_marker.corners
        if len(corners) > 0:
            _ids = []
            _corners = []
            for i in range(len(corners)):
                _ids.append(ids[i])
                _corners.append(np.array([[[corners[i].top_left.x, corners[i].top_left.y],
                                          [corners[i].top_right.x, corners[i].top_right.y],
                                          [corners[i].bottom_right.x, corners[i].bottom_right.y],
                                          [corners[i].bottom_left.x, corners[i].bottom_left.y]]], np.float32))
            self.get_logger().info(f'[ ARUCO DRAWER NODE ]: my_corners = {tuple(_corners)}')
            self.get_logger().info(f'[ ARUCO DRAWER NODE ]: my_corners_len = {len(tuple(_corners))}')
            self.get_logger().info(f'[ ARUCO DRAWER NODE ]: ids = {_ids}')
            self.image_markers_drawed = aruco.drawDetectedMarkers(self.image_camera, tuple(_corners), np.array([_ids], np.float32))
            
            # publish
            msg = self.bridge.cv2_to_imgmsg(self.image_markers_drawed, encoding="passthrough")
            self.publisher.publish(msg)
        else:
            self.image_markers_drawed = None


def main():
    rclpy.init()
    node = ArucoDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
