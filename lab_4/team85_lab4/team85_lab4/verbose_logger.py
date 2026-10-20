from rclpy.node import Node


# Levels of the "verbose" parameter
QUIET = 0       # No logging
BASIC = 1       # One-off messages (startup, state changes)
CONTINUOUS = 2  # Also periodic results (obstacle position, goal vector, ...)


class VerboseLogger:
    """
    Info logging gated by the node's "verbose" parameter. The parameter is read
    on every call, so it can be changed with `ros2 param set`.

    Warnings and errors should go through node.get_logger() and are never gated.
    """

    def __init__(self, node: Node, default: int = BASIC):
        self.node = node
        node.declare_parameter("verbose", default)

    @property
    def level(self) -> int:
        return int(self.node.get_parameter("verbose").value)

    def enabled(self, level: int) -> bool:
        return self.level >= level

    def basic(self, message: str, **kwargs):
        if self.enabled(BASIC):
            self.node.get_logger().info(message, **kwargs)

    def continuous(self, message: str, **kwargs):
        if self.enabled(CONTINUOUS):
            self.node.get_logger().info(message, **kwargs)
