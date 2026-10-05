from keyboard_interface.keyboard_input_node import KeyboardInterfaceNode
import rclpy
from rclpy.executors import ExternalShutdownException


def main() -> None:
    rclpy.init()
    node = KeyboardInterfaceNode()
    node.start_listener()
    node.get_logger().info('keyboard_interface_node started.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_listener()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
