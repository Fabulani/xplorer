#!/usr/bin/env python3

#  ------------------------------------------------------------------
#   Copyright (C) Karelics Oy - All Rights Reserved
#   Unauthorized copying of this file, via any medium is strictly
#   prohibited. All information contained herein is, and remains
#   the property of Karelics Oy.
#  ------------------------------------------------------------------

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class DemoPublisher(Node):
    def __init__(self):
        super().__init__("demo_publisher")

        # here we are showing how the launch arguments/parameters can be used if needed
        self.declare_parameter("dummy_arg_1", 0.0)
        dummy_arg_1 = float(self.get_parameter("dummy_arg_1").value)  # here we have to explicitly cast to float
        self.get_logger().info(f"the value of dummy_arg_1 is {dummy_arg_1}")

        self.declare_parameter("dummy_arg_2", "default string")
        dummy_arg_2 = self.get_parameter(
            "dummy_arg_2"
        ).value  # here we do not cast as the type of this param is a string, and we get it already like that
        self.get_logger().info(f"the value of dummy_arg_2 is {dummy_arg_2}")

        self._publisher = self.create_publisher(Twist, "/cmd_vel/safe", 1)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        self._publisher.publish(msg)
        self.get_logger().info("Publishing twist")


def main(args=None):
    rclpy.init(args=args)

    demo_publisher = DemoPublisher()

    try:
        rclpy.spin(demo_publisher)
    except KeyboardInterrupt:
        pass

    demo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
