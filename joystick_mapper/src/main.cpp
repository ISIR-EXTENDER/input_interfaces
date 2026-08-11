#include "joystick_mapper/joystick_mapper.hpp"

#include <memory>

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joystick_mapper::JoystickMapper>());
  rclcpp::shutdown();
  return 0;
}
