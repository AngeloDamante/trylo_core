#!/usr/bin/env python

"""Control Node"""

import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Vector3
# from src.trylo_motor.msg import Command
from trylo_motor.msg import Command
from src.trylo_motor.trylo_motor.Commands import Commands
# from src.trylo_gpio.trylo_gpio.Trylo import Trylo

SPEED_TURN = 0.5
SPEED_FF = 0.7

class Control(Node):
    def __init__(self):
        super().__init__("Controller")

        # ros2 attributes
        self.subscription = self.create_subscription(Vector3, '/ref/coord', self.cbk_read_direction, 10)
        self.publisher = self.create_publisher(Command, '/motor/speed', 10)
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

        # computing phases
        if self.target.z == 0.0:
            self.command = Commands.STOP
            self.speed = 0.0
        else:
            _angle = math.atan(self.target.x / self.target.z)
            if _angle > 0.0:
                self.command = Commands.TURN_RIGHT
                self.speed = SPEED_TURN
            elif _angle < 0.0:
                self.command = Commands.TURN_LEFT
                self.speed = SPEED_TURN
            elif _angle == 0.0:
                self.command = Commands.FORWARD
                self.speed = SPEED_FF            

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
