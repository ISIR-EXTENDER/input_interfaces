#include "rclcpp/rclcpp.hpp"
#include "joystick_interface/snake_demo.hpp"

int main(int argc, char * argv[])
{
  // Initialize the ROS2 client library.
  rclcpp::init(argc, argv);

  // Create an instance of TeleopController.
  auto joystick_controller_node = std::make_shared<input_interfaces::JoystickSnake>();

  // Spin the node so that callbacks are processed.
  rclcpp::spin(joystick_controller_node);

  rclcpp::shutdown();
  return 0;
}
