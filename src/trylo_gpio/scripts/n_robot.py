#!/usr/bin/env python
"""Robot Handler"""

import rclpy
from rclpy.node import Node
from trylo_gpio.msg import Command
from trylo_gpio.Trylo import Trylo
from std_msgs.msg import Float32
from src.trylo_gpio.trylo_gpio.Directive import Directive


class Motor(Node):
    def __init__(self):
        super().__init__("n_robot")
        self.get_logger().info(f'[ ROBOT NODE ]: init {self.get_name()}')
        self.subscription = self.create_subscription(Command, '/gpio/motor', self.cbk_read_command, 10)
        # self.publisher = self.create_publisher(Float32, '/gpio/us_distance', 5)
        # self.create_timer(0.1, self.cbk_pub_distance) #FIXME
        self.robot = Trylo()

    def cbk_pub_distance(self):
        msg = Float32()
        msg.data = self.robot.read_distance()
        self.publisher.publish(msg)

    def cbk_read_command(self, cmd):
        _cmd, _speed = cmd.command, cmd.speed
        self.get_logger().info(f'[ ROBOT NODE ]: parsing command = {cmd.command}, speed={cmd.speed}')
        if _cmd == Directive.FORWARD.value:
            self.get_logger().info('[ ROBOT NODE ]: FORWARD')
            self.robot.forward(speed=_speed)
        elif _cmd == Directive.BACKWARD.value:
            self.get_logger().info('[ ROBOT NODE ]: BACKWARD')
            self.robot.backward(speed=_speed)
        elif _cmd == Directive.TURN_RIGHT.value:
            self.get_logger().info('[ ROBOT NODE ]: TURN RIGHT')
            self.robot.turn_right(speed=_speed)
        elif _cmd == Directive.TURN_LEFT.value:
            self.get_logger().info('[ ROBOT NODE ]: TURN LEFT')
            self.robot.turn_left(speed=_speed)
        elif _cmd == Directive.CURVE_RIGHT.value:
            self.get_logger().info('[ ROBOT NODE ]: CURVE RIGHT')
            self.robot.curve_forward_right(speed=_speed)
        elif _cmd == Directive.CURVE_LEFT.value:
            self.get_logger().info('[ ROBOT NODE ]: CURVE LEFT')
            self.robot.curve_forward_left(speed=_speed)
        elif _cmd == Directive.STOP.value:
            self.get_logger().info('[ ROBOT NODE ]: STOP')
            self.robot.stop()
        elif _cmd == Directive.COAST.value:
            self.get_logger().info('[ ROBOT NODE ]: COAST')
            self.robot.coast()


def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
