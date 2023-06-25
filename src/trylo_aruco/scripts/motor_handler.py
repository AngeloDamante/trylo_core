#!/usr/bin/env python

"""Motor Handler"""

import rclpy
from rclpy.node import Node
from trylo_motor.msg import Command
from src.trylo_motor.trylo_motor.Commands import Commands
from src.trylo_gpio.trylo_gpio.Trylo import Trylo


class MotorHandler(Node):
    def __init__(self):
        super().__init__("Motor_Handler")
        self.subscription = self.create_subscription(Command, '/motor/speed', self.cbk_read_command, 10)
        self.robot = Trylo()

    def cbk_read_command(self, cmd):
        if cmd.command == Commands.FORWARD.value:
            self.robot.forward(speed=cmd.speed)
        elif cmd.command == Commands.BACKWARD.value:
            self.robot.backward(speed=cmd.speed)
        elif cmd.command == Commands.TURN_RIGHT.value:
            self.robot.turn_right(speed=cmd.speed)
        elif cmd.command == Commands.TURN_LEFT.value:
            self.robot.turn_left(speed=cmd.speed)
        elif cmd.command == Commands.CURVE_RIGHT.value:
            self.robot.curve_forward_right(speed=cmd.speed)
        elif cmd.command == Commands.CURVE_LEFT.value:
            self.robot.curve_forward_left(speed=cmd.speed)
        elif cmd.command == Commands.STOP.value:
            self.robot.stop()
        elif cmd.command == Commands.COAST.value:
            self.robot.coast()


def main(args=None):
    rclpy.init(args=args)
    node = MotorHandler()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
