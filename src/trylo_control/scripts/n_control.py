#!/usr/bin/env python
"""Control Node"""

import rclpy
from typing import Tuple
from rclpy.node import Node

# packages
from src.trylo_control.trylo_control.utils import remap

# messages
from trylo_gpio.msg import Command
from trylo_control.msg import Reference
from std_msgs.msg import Float32

# params
from src.definitions import Directive
from src.parameters import(
    DEG_RANGE,
    D_MIN,
    D_MAX,
    SPEED_MIN,
    SPEED_MAX
)

class Control(Node):
    def __init__(self, deg_range: tuple, d_min: float, d_max: float, speed_min: float, speed_max: float):
        super().__init__("n_control")
        self.get_logger().info(f'[ CONTROL NODE ]: init {self.get_name()}')

        # publish
        self.publisher = self.create_publisher(Command, "/gpio/motor", 10)
        self.create_timer(0.02, self.cbk_pub_cmd) 

        # subscribers
        self.subscriber_obstacle = self.create_subscription(Float32, "/gpio/us_distance", self.cbk_get_distance, 10)
        self.subscriber_refgen = self.create_subscription(Reference, "/control/ref", self.cbk_get_ref, 10)

        # attributes
        self.deg_range = deg_range
        self.d_min = d_min
        self.d_max = d_max
        self.speed_min = speed_min
        self.speed_max = speed_max

        # hcsr04 sensor
        self.obstacle = False

        # init command
        self.cmd = Command()
        self.cmd.command = Directive.STOP.value
        self.cmd.speed = 0.0

    def cbk_pub_cmd(self):
        self.publisher.publish(self.cmd)

    def cbk_get_distance(self, msg):
        distance_hcsr04 = msg.data / 100. # to meters
        self.get_logger().info(f"[ CONTROL NODE ]: {distance_hcsr04}")
        if distance_hcsr04 < self.d_min:
            self.get_logger().info("[ CONTROL NODE ]: obstacle!!")
            self.obstacle = True
            self.cmd.command = Directive.STOP.value
            self.cmd.speed = 0.0
        else:
            self.obstacle = False

    def cbk_get_ref(self, msg):
        _cmd, _speed = self.chose_command(msg.theta, msg.distance)
        self.get_logger().info(f"[ CONTROL NODE ]: handle  cmd = {_cmd} speed = {_speed}")
        self.cmd.command = _cmd
        self.cmd.speed = _speed

    def chose_command(self, ref_theta: float, ref_d: float) -> Tuple[Directive, float]:
        self.get_logger().info(f'[ CONTROL NODE ]: parsing theta = {ref_theta}, d = {ref_d}')
        if ref_d < self.d_min or self.obstacle is True:
            self.get_logger().info('[ CONTROL NODE ]: Obstacle or target reached')
            cmd = Directive.STOP.value
            speed = 0.0
        elif self.deg_range[0] < ref_theta < self.deg_range[1]:
            self.get_logger().info('[ CONTROL NODE ]: theta aligned, go ahead')
            cmd = Directive.FORWARD.value
            speed = remap(ref_d, self.d_min, self.d_max, self.speed_min, self.speed_max)
        elif ref_theta > 0:
            self.get_logger().info('[ CONTROL NODE ]: RIGHT')
            cmd = Directive.CURVE_RIGHT.value
            speed = 0.4
        elif ref_theta < 0:
            self.get_logger().info('[ CONTROL NODE ]: LEFT')
            cmd = Directive.CURVE_LEFT.value
            speed = 0.4
        return cmd, speed


def main(args=None):
    rclpy.init(args=args)
    node = Control(deg_range=DEG_RANGE, 
                   d_min=D_MIN, 
                   d_max=D_MAX, 
                   speed_min=SPEED_MIN, 
                   speed_max=SPEED_MAX)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
