#!/usr/bin/env python

"""Generate PWM desired"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class PWMGen(Node):
    def __init__(self):
        super().__init__("pwm_gen")
        self.declare_parameter('pwm', value=0.0)
        self.publisher = self.create_publisher(Float32, "/motor/pwm", 10)
        self.timer = self.create_timer(timer_period_sec=0.02, callback=self.callback_pwm_gen)

    def callback_pwm_gen(self):
        msg = Float32()
        # msg.data = 0.2
        msg.data = self.get_parameter('pwm').value
        self.publisher.publish(msg=msg)
        self.get_logger().info(f"[ PWM_PUB = {msg.data} ]")


def main(args=None):
    rclpy.init(args=args)
    node = PWMGen()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()