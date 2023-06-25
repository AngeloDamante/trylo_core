#!/usr/bin/env python

"""Control Node"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Vector3
from src.trylo_motor.msg import Command
from src.trylo_motor.trylo_motor.Commands import Commands
from src.trylo_gpio.trylo_gpio.Trylo import Trylo


class Control(Node):
    def __init__(self):
        super().__init__("Controller")

        # ros2 attributes
        self.subscription = self.create_subscription(Vector3, '/ref/coord', self.cbk_read_direction, 10)
        self.publisher = self.create_publisher(Command, '/motor/spped', 10)
        self.timer = self.create_timer(timer_period_sec=0.02, callback=self.cbk_write_command)

        # control
        self.target = None
        self.command = None
        self.speed = None

    def cbk_read_direction(self, target):
        self.target = target
        self.get_logger().info(f"[ x = {self.target.x}, y = {self.target.y}, z = {self.target.z} ]")

    def cbk_write_command(self):
        if self.target is None:
            return
        _angle = math.atan(self.target.x / self.target.z)
        if _angle > 0:
            self.command = Command.TURN_RIGHT
            self.speed = 0.2
        elif _angle < 0:
            self.command = Command.TURN_LEFT
            self.speed = 0.2
        elif _angle == 0:
            self.command = Command.FORWARD
            self.speed = 0.7

        # msg
        msg = Command()
        msg.command = self.command.value
        msg.speed = self.speed
        self.publisher.publish(msg=msg)


def main(args=None):
    rclpy.init(args=args)
    node = Control()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
