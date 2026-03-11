from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

try:
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore


class RawToCompressedBridge(Node):
    def __init__(self) -> None:
        super().__init__("raw_to_compressed_bridge")

        self.declare_parameter("input_image_topic", "/camera/color/image_raw")
        self.declare_parameter(
            "output_compressed_topic",
            "/petanque/measure/request_image/compressed",
        )
        self.declare_parameter("jpeg_quality", 90)

        input_topic = str(self.get_parameter("input_image_topic").value)
        output_topic = str(self.get_parameter("output_compressed_topic").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self._publisher = self.create_publisher(CompressedImage, output_topic, 10)
        self._subscription = self.create_subscription(
            Image,
            input_topic,
            self._on_image,
            10,
        )

        self.get_logger().info(
            "Raw->Compressed bridge started: in={0} out={1} jpeg_quality={2}".format(
                input_topic,
                output_topic,
                self._jpeg_quality,
            )
        )

    def _on_image(self, msg: Image) -> None:
        if cv2 is None:
            return

        image_bgr = self._image_msg_to_bgr(msg)
        if image_bgr is None:
            return

        ok, encoded = cv2.imencode(
            ".jpg",
            image_bgr,
            [cv2.IMWRITE_JPEG_QUALITY, max(1, min(100, self._jpeg_quality))],
        )
        if not ok:
            self.get_logger().warning("Failed to encode compressed image")
            return

        out = CompressedImage()
        out.header = msg.header
        out.format = "jpeg"
        out.data = encoded.tobytes()
        self._publisher.publish(out)

    def _image_msg_to_bgr(self, msg: Image) -> np.ndarray | None:
        if msg.height <= 0 or msg.width <= 0:
            return None

        data = np.frombuffer(msg.data, dtype=np.uint8)
        channels = (
            3
            if msg.encoding in {"rgb8", "bgr8"}
            else 4 if msg.encoding in {"rgba8", "bgra8"} else 0
        )
        if channels == 0:
            return None

        expected_size = int(msg.height) * int(msg.width) * channels
        if data.size < expected_size:
            return None

        frame = data[:expected_size].reshape((int(msg.height), int(msg.width), channels))
        if msg.encoding == "bgr8":
            return frame.copy()
        if msg.encoding == "rgb8":
            return frame[:, :, ::-1].copy()
        if msg.encoding == "bgra8":
            return frame[:, :, :3].copy()
        if msg.encoding == "rgba8":
            return frame[:, :, :3][:, :, ::-1].copy()
        return None


def main() -> None:
    rclpy.init()
    node = RawToCompressedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
