#!/usr/bin/env python
"""Test reference"""

import cv2
import numpy as np
from typing import Tuple

# ros2
import rclpy
from rclpy.node import Node

# messages
from trylo_control.msg import Reference


class ToyRef(Node):
    def __init__(self):
        super().__init__("n_tst_ref")

        self.publisher = self.create_publisher(Reference, "/control/ref", 5)
        self.create_timer(0.02, self.cbk_pub_ref)
        
        self.theta_ref = 0.0
        self.d_ref = 0.6
        
    def cbk_pub_ref(self):
        msg = Reference()
        msg.theta = self.theta_ref
        msg.distance = self.d_ref
        self.publisher.publish(msg)

   

def main(args=None):
    rclpy.init(args=args)
    node = ToyRef()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
