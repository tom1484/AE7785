#!/usr/bin/env python3

"""
CS/ME/ECE/AE/BME 7785 - Lab 5.

Authors:
Chu-Rong Chen
Xingyu Zhu

Node:
    waypoint_navigator

Reads the goal points from wayPoints.txt and sends them to Nav2 one at a time.
Not implemented yet: this is the package skeleton.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__("waypoint_navigator")

        # TODO: read the waypoints from wayPoints.txt.
        # TODO: send each one to Nav2 (/goal_pose or the NavigateToPose action)
        #       and wait until it is reached before sending the next.
        self.get_logger().warn("waypoint_navigator is not implemented yet")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
