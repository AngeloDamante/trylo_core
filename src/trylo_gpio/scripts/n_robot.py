#!/usr/bin/env python
"""Robot Handler"""

# ros2
import rclpy
from rclpy.node import Node

# msg
from std_msgs.msg import Float32
from trylo_gpio.msg import Command, CmdLed

# lib
from trylo_gpio.Trylo import Trylo
from trylo_gpio.names import LIGHTS_FRONT, LIGHTS_RIGHT, LIGHTS_LEFT
from src.definitions import Directive, Led


class Motor(Node):
    def __init__(self):
        super().__init__("n_robot")
        self.get_logger().info(f'[ ROBOT NODE ]: init {self.get_name()}')
        self.motor_sub = self.create_subscription(Command, '/gpio/motor', self.cbk_read_command, 10)
        self.led_sub = self.create_subscription(CmdLed, '/gpio/led', self.cbk_read_led, 10)
        self.robot = Trylo()

    def cbk_pub_distance(self):
        msg = Float32()
        msg.data = self.robot.read_distance()
        self.publisher.publish(msg)

    def cbk_read_led(self, cmd_led):
        _cmd_led, _color = cmd_led.func, (cmd_led.r, cmd_led.g, cmd_led.b) 
        if _cmd_led == Led.FILL.value:
            self.robot.fill_underlighting(color=_color)
        elif _cmd_led == Led.CLEAR.value:
            self.robot.clear_underlighting()
        elif _cmd_led == Led.FRONT_SIDE.value:
            self.robot.set_underlights(lights=LIGHTS_FRONT, color=_color)
        elif _cmd_led == Led.LEFT_SIDE.value:
            self.robot.set_underlights(lights=LIGHTS_LEFT, color=_color)
        elif _cmd_led == Led.RIGHT_SIDE.value:
            self.robot.set_underlights(lights=LIGHTS_RIGHT, color=_color)

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
