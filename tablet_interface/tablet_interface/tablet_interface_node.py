import rclpy
from rclpy.node import Node

from extender_msgs.msg import TeleopCommand


class TabletInterfaceNode(Node):
    def __init__(self) -> None:
        super().__init__("tablet_interface_node")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("start_mode", "TRANSLATION_ROTATION")

        self._publisher = self.create_publisher(TeleopCommand, "/teleop_cmd", 10)

        rate = self.get_parameter("publish_rate_hz").value
        self._timer = self.create_timer(1.0 / float(rate), self._on_timer)

        self.get_logger().info("Tablet interface node initialized")

    def _on_timer(self) -> None:
        msg = TeleopCommand()
        msg.mode = TeleopCommand.TRANSLATION_ROTATION
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = TabletInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
