#!/usr/bin/env python3

import math
import os

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped

from turtlebot3_msgs.srv import Sound
from turtlebot3_recovery_lab.srv import DivertRobot


# TurtleBot3 sound commands:
# 0 = OFF
# 1 = ON
# 2 = LOW_BATTERY
# 3 = ERROR
# 4 = BUTTON1
# 5 = BUTTON2
SOUND_ERROR = 3


class StudentRecoveryServer(Node):
    def __init__(self):
        super().__init__('student_recovery_server')

        default_file = os.path.join(
            get_package_share_directory('turtlebot3_recovery_lab'),
            'config',
            'recovery_candidates.yaml'
        )
        candidates_file = self.declare_parameter(
            'candidates_file', default_file
        ).value
        # Sound played once a recovery waypoint has been found. Must differ
        # from SOUND_ERROR, which is played when recovery starts.
        self.success_sound = self.declare_parameter('success_sound', 1).value

        self.frame_id, self.min_distance, self.candidates = (
            self.load_candidates(candidates_file)
        )

        # Latest robot position (x, y) in the map frame, from AMCL
        self.robot_xy = None

        # ID of the Nav2 goal currently executing, from the action status
        self.active_goal_id = None

        # Last diversion: (goal ID, candidate index). If Nav2 asks for another
        # recovery during the same goal, that candidate didn't fix the problem.
        self.last_divert = None

        # Candidates that failed during the current goal
        self.failed = set()

        # TurtleBot3 OpenCR buzzer service client
        self.sound_client = self.create_client(
            Sound,
            '/sound'
        )

        # AMCL and the action status topic both publish as transient local,
        # so the latest message arrives even if it was sent before we started.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            latched
        )

        self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_status_callback,
            latched
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

    def load_candidates(self, path):
        """Read the frame, minimum distance and candidate points from YAML."""

        with open(path, 'r') as f:
            config = yaml.safe_load(f)

        frame_id = config.get('frame_id', 'map')
        min_distance = float(config.get('min_distance', 0.3))
        candidates = [
            (float(c['x']), float(c['y'])) for c in config['candidates']
        ]

        if not candidates:
            raise ValueError(f'No recovery candidates in {path}')

        self.get_logger().info(
            f'Loaded {len(candidates)} recovery candidates from {path}'
        )
        return frame_id, min_distance, candidates

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        self.robot_xy = (position.x, position.y)

    def goal_status_callback(self, msg):
        # The goal that is executing now. Nav2 runs one NavigateToPose goal
        # at a time, so take the last one if there are several.
        active = [
            s for s in msg.status_list
            if s.status in (GoalStatus.STATUS_ACCEPTED,
                            GoalStatus.STATUS_EXECUTING)
        ]
        if active:
            self.active_goal_id = bytes(active[-1].goal_info.goal_id.uuid).hex()

    def beep(self, value=SOUND_ERROR):
        """Trigger the TurtleBot3 buzzer asynchronously."""

        if not self.sound_client.service_is_ready():
            self.get_logger().warn(
                'Cannot beep: /sound service unavailable'
            )
            return

        sound_request = Sound.Request()
        sound_request.value = value

        try:
            future = self.sound_client.call_async(sound_request)
            future.add_done_callback(self.sound_response_callback)

            self.get_logger().info(
                f'Sent sound {value} request to TurtleBot3'
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

    def update_failed_candidates(self):
        """Exclude the last candidate if it didn't fix the current goal."""

        goal_id = self.active_goal_id

        if goal_id is None:
            self.get_logger().warn(
                'No Nav2 goal status received; treating this as the same '
                'goal as the last recovery'
            )

        if self.last_divert is None:
            return

        last_goal_id, last_index = self.last_divert

        if goal_id == last_goal_id:
            # Stuck again on the same goal after diverting to this candidate
            self.failed.add(last_index)
            x, y = self.candidates[last_index]
            self.get_logger().warn(
                f'Candidate {last_index} ({x:.2f}, {y:.2f}) did not fix the '
                f'problem; excluding it for this goal'
            )
        else:
            # New goal: every candidate gets another chance
            self.failed.clear()

    def choose_candidate(self):
        """Return the index of the best candidate, or None if none is usable."""

        usable = [
            i for i in range(len(self.candidates)) if i not in self.failed
        ]

        if self.robot_xy is None:
            self.get_logger().warn(
                'No /amcl_pose received yet; using the first usable '
                'candidate in the file'
            )
            return usable[0] if usable else None

        def distance(i):
            return math.dist(self.robot_xy, self.candidates[i])

        far_enough = [i for i in usable if distance(i) >= self.min_distance]
        if not far_enough:
            return None

        return min(far_enough, key=distance)

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

        self.beep(SOUND_ERROR)

        # --------------------------------------------------
        # STEP 2: Compute recovery waypoint
        # --------------------------------------------------

        # Nearest candidate to the robot that is at least min_distance away
        # and hasn't already failed during this goal.
        self.update_failed_candidates()
        index = self.choose_candidate()

        response.recovery_pose.header.frame_id = self.frame_id
        response.recovery_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )
        response.recovery_pose.pose.orientation.w = 1.0

        # --------------------------------------------------
        # STEP 3: Return recovery waypoint
        # --------------------------------------------------

        if index is None:
            response.success = False
            self.get_logger().error(
                f'No usable recovery candidate (robot at {self.robot_xy}, '
                f'{len(self.failed)} of {len(self.candidates)} excluded)'
            )
            return response

        recovery_x, recovery_y = self.candidates[index]
        response.recovery_pose.pose.position.x = recovery_x
        response.recovery_pose.pose.position.y = recovery_y
        response.recovery_pose.pose.position.z = 0.0
        response.success = True

        self.last_divert = (self.active_goal_id, index)

        self.get_logger().info(
            f'Recovery waypoint generated: candidate {index} '
            f'({recovery_x:.2f}, {recovery_y:.2f})'
        )

        # ---------------------------------------------------
        # STEP 4: Trigger a different TurtleBot3 buzzer if a
        # recovery point was found
        # ---------------------------------------------------

        self.beep(self.success_sound)

        return response


def main(args=None):
    rclpy.init(args=args)

    node = StudentRecoveryServer()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
