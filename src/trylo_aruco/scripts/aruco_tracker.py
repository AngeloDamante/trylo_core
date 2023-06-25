#!/usr/bin/env python

"""Aruco Tracker Node"""  # TODO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3


class FakeNode(Node):
    def __init__(self):
        super().__init__("Fake_gen")
        self.declare_parameter('x', value=0.0)
        self.declare_parameter('y', value=0.0)
        self.declare_parameter('z', value=0.0)
        self.publisher = self.create_publisher(msg_type=Vector3, topic='/ref/coord', qos_profile=10)
        self.timer = self.create_timer(timer_period_sec=0.3, callback=self.cbk_generate_fake_ref)

    def cbk_generate_fake_ref(self):
        msg = Vector3()
        msg.x = self.get_parameter('x').value
        msg.y = self.get_parameter('y').value
        msg.z = self.get_parameter('z').value
        self.publisher.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
