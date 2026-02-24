import threading

import rclpy

from tablet_interface.ros_teleop_publisher import TabletInterfaceNode
from tablet_interface.ws_server import run_uvicorn_server


def main() -> None:
    rclpy.init()
    node = TabletInterfaceNode()
    t = threading.Thread(target=run_uvicorn_server, args=(node,), daemon=True)
    t.start()
    node.get_logger().info(
        "tablet_interface_node started (ROS spinning + WS server thread)."
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
