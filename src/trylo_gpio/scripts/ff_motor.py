#!/usr/bin/env python

"""Generate Feed Forward Control"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from trylo_gpio.Trylo import Trylo


class FeedForward(Node):
    def __init__(self):
        super().__init__("ff_motor")
        self.subscription = self.create_subscription(Float32, "/motor/pwm", self.callback_ff, 10)
        self.robot = Trylo()

    def callback_ff(self, msg):
        self.robot.forward(msg.data)
        self.get_logger().info(f"[ PWM_SUB = {msg.data} ]")


def main(args=None):
    rclpy.init(args=args)
    node = FeedForward()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
