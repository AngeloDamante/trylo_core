#!/usr/bin/env python
"""To Test Motor Handler"""

import rclpy
from rclpy.node import Node
from trylo_gpio.msg import Command
from src.definitions import Directive

class PubCommand(Node):
    def __init__(self):
        super().__init__("n_pub_cmd")
        self.get_logger().info(f'[ PUB_CMD ]: init {self.get_name()}')
        self.publisher = self.create_publisher(Command, "/gpio/motor", 10)
        self.create_timer(0.02, self.cbk_pub_cmd)
        
    def cbk_pub_cmd(self):
        msg = Command()
        msg.command = Directive.FORWARD.value
        msg.speed = 0.7
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PubCommand()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()