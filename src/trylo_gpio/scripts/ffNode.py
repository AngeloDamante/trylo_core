#!/usr/bin/env python

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trylo_gpio.Trylo import Trylo
from trylo_gpio.names import NUM_UNDERLIGHTS


class FeedForward(Node):
    def __init__(self):
        super().__init__("ff_node")
        self.publisher = self.create_publisher(String, "control_value", 5)
        self.robot = Trylo()
        self.timer = self.create_timer(0.5, self.callback_ff)

    def callback_ff(self):
        msg = String()

        self.robot.fill_underlighting((127, 127, 127), show=True)
        self.robot.forward(1.0)
        # for led in range(NUM_UNDERLIGHTS):
        #     self.robot.clear_underlighting(show=False)
        #     time.sleep(0.2)
        #     self.robot.set_underlight(led, (255, 0, 0))
        #     time.sleep(0.2)
        #     self.robot.clear_underlighting(show=False)
        #     time.sleep(0.2)
        #     self.robot.set_underlight(led, (0, 255, 0))
        #     time.sleep(0.2)
        #     self.robot.clear_underlighting(show=False)
        #     time.sleep(0.2)
        #     self.robot.set_underlight(led, (0, 0, 255))
        # self.robot.clear_underlights()

        msg.data = f'PUBLISH'
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FeedForward()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
