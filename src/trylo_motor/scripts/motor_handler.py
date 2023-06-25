#!/usr/bin/env python

"""Motor Handler"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from src.trylo_gpio.trylo_gpio.Trylo import Trylo


class MotorHandler(Node):
    def __init__(self):
        super().__init__("motor_handler")
        self.subscription = self.create_subscription(Float32, "/motor/pwm", self.callback_motor, 10)
        self.robot = Trylo()

    def callback_motor(self, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MotorHandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
