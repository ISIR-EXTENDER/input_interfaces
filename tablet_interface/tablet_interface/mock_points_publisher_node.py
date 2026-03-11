from __future__ import annotations

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MockPointsPublisher(Node):
    def __init__(self) -> None:
        super().__init__("petanque_mock_points_publisher")

        self.declare_parameter("points_topic", "/petanque_measurements/points")
        self.declare_parameter("publish_rate_hz", 5.0)
        numeric_arg_descriptor = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("point_ax", 220.0, numeric_arg_descriptor)
        self.declare_parameter("point_ay", 260.0, numeric_arg_descriptor)
        self.declare_parameter("point_bx", 420.0, numeric_arg_descriptor)
        self.declare_parameter("point_by", 260.0, numeric_arg_descriptor)

        points_topic = str(self.get_parameter("points_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self._point_a = (
            float(self.get_parameter("point_ax").value),
            float(self.get_parameter("point_ay").value),
        )
        self._point_b = (
            float(self.get_parameter("point_bx").value),
            float(self.get_parameter("point_by").value),
        )

        self._publisher = self.create_publisher(Float32MultiArray, points_topic, 10)
        self._timer = self.create_timer(1.0 / max(publish_rate_hz, 1e-3), self._on_timer)

        self.get_logger().info(
            "Mock points publisher started: topic={0} point_a=({1:.1f},{2:.1f}) "
            "point_b=({3:.1f},{4:.1f}) rate={5:.1f}Hz".format(
                points_topic,
                self._point_a[0],
                self._point_a[1],
                self._point_b[0],
                self._point_b[1],
                publish_rate_hz,
            )
        )

    def _on_timer(self) -> None:
        msg = Float32MultiArray()
        msg.data = [
            float(self._point_a[0]),
            float(self._point_a[1]),
            float(self._point_b[0]),
            float(self._point_b[1]),
        ]
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = MockPointsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
