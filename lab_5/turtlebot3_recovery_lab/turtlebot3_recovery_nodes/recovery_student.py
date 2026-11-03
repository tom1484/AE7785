#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from turtlebot3_msgs.srv import Sound
from turtlebot3_recovery_lab.srv import DivertRobot


class StudentRecoveryServer(Node):
    def __init__(self):
        super().__init__('student_recovery_server')

        # TurtleBot3 OpenCR buzzer service client
        self.sound_client = self.create_client(
            Sound,
            '/sound'
        )

        # Recovery service called by the C++ Behavior Tree bridge
        self.srv = self.create_service(
            DivertRobot,
            '/divert_robot',
            self.handle_recovery
        )

        self.get_logger().info(
            'Recovery Python Server listening on /divert_robot'
        )

        if self.sound_client.service_is_ready():
            self.get_logger().info(
                'TurtleBot3 sound service available'
            )
        else:
            self.get_logger().warn(
                'TurtleBot3 sound service not currently available'
            )

    def beep(self):
        """Trigger the TurtleBot3 error buzzer asynchronously."""

        if not self.sound_client.service_is_ready():
            self.get_logger().warn(
                'Cannot beep: /sound service unavailable'
            )
            return

        sound_request = Sound.Request()

        # TurtleBot3 sound commands:
        # 0 = OFF
        # 1 = ON
        # 2 = LOW_BATTERY
        # 3 = ERROR
        # 4 = BUTTON1
        # 5 = BUTTON2
        sound_request.value = 3

        try:
            future = self.sound_client.call_async(sound_request)
            future.add_done_callback(self.sound_response_callback)

            self.get_logger().info(
                'Sent ERROR sound request to TurtleBot3'
            )

        except Exception as exc:
            self.get_logger().error(
                f'Failed to send sound request: {exc}'
            )

    def sound_response_callback(self, future):
        """Process the asynchronous sound service response."""

        try:
            result = future.result()

            if result.success:
                self.get_logger().info(
                    'TurtleBot3 buzzer activated successfully'
                )
            else:
                self.get_logger().warn(
                    f'TurtleBot3 buzzer failed: {result.message}'
                )

        except Exception as exc:
            self.get_logger().error(
                f'Sound service call failed: {exc}'
            )

    def handle_recovery(self, request, response):
        """
        Called when Nav2 triggers the custom recovery behavior.

        1. Trigger the TurtleBot3 buzzer.
        2. Compute a recovery waypoint.
        3. Return the waypoint to the C++ BT bridge if one is available.
        4. Trigger a different Turtlebot3 buzzer if successful.
        """

        self.get_logger().warn(
            'Nav2 stalled! Sounding alarm and calculating diversion route...'
        )

        # --------------------------------------------------
        # STEP 1: Trigger the TurtleBot3 buzzer
        # --------------------------------------------------

        self.beep()

        # --------------------------------------------------
        # STEP 2: Compute recovery waypoint
        # --------------------------------------------------

        # TODO 1: Choose a recovery waypoint.
        #
        # Replace the placeholder values below with your recovery-planning
        # algorithm. Your algorithm should select a safe, reachable waypoint
        # that helps the robot recover from being stuck.
        #
        # Hints:
        #   - The waypoint coordinates should be expressed in the 'map' frame.
        #   - Consider how you would avoid choosing the robot's current location
        #     or a location that is likely to be blocked.
        #   - For this lab, you may start with a simple fixed waypoint, then
        #     extend the logic using information available in `request`.
        #
        # TODO 2: Set recovery_x and recovery_y to your chosen waypoint.
        recovery_x = ...  # TODO: x-coordinate (meters)
        recovery_y = ...  # TODO: y-coordinate (meters)

        response.recovery_pose.header.frame_id = 'map'
        response.recovery_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        response.recovery_pose.pose.position.x = recovery_x
        response.recovery_pose.pose.position.y = recovery_y
        response.recovery_pose.pose.position.z = 0.0

        response.recovery_pose.pose.orientation.x = 0.0
        response.recovery_pose.pose.orientation.y = 0.0
        response.recovery_pose.pose.orientation.z = 0.0
        response.recovery_pose.pose.orientation.w = 1.0

        # --------------------------------------------------
        # STEP 3: Return recovery waypoint
        # --------------------------------------------------

        # TODO 3: Set response.success based on whether a valid recovery
        # waypoint was found. Do not report success if planning failed.
        response.success = True

        self.get_logger().info(
            f'Recovery waypoint generated: '
            f'({recovery_x:.2f}, {recovery_y:.2f})'
        )

        # ---------------------------------------------------
        # STEP 4: Trigger a different TurtleBot3 buzzer if a
        # recovery point was found
        # ---------------------------------------------------

        return response


def main(args=None):
    rclpy.init(args=args)

    node = StudentRecoveryServer()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()