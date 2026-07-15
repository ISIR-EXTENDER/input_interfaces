#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "extender_msgs/msg/teleop_command.hpp"
#include "std_msgs/msg/bool.hpp"

namespace input_interfaces
{
  /// @brief Node that will subscribe either to a SpaceMouse or a normal joystick, and send out
  /// teloep_cmd messages (twist + mode of command) on the topic /teleop_cmd
  class JoystickSnake : public rclcpp::Node
  {
  public:
    /// @brief Constructor of the class. Will intialize parameters as well a subscriber and
    /// publisher.
    JoystickSnake();

  private:
    /**
     * @brief Template function to declare and get parameters with default values.
     * @tparam T Type of the parameter.
     * @param name Parameter name.
     * @param variable Reference to store the parameter value.
     * @param default_value Default value if parameter is not set.
     */
    template <typename T>
    void declare_and_get_parameters(const std::string &name, T &variable, const T &default_value)
    {
      if (!this->has_parameter(name))
      {
        this->declare_parameter(name, default_value);
      }
      variable = this->get_parameter(name).get_value<T>();
    }

    /// @brief Callback function to process incoming Joy messages from the 3D joystick.
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief The current teleoperation mode. Defaults to TRANSLATION_ROTATION.
    uint8_t current_mode_{extender_msgs::msg::TeleopCommand::TRANSLATION_ROTATION};

    /// @brief Subscriber for receiving Joy messages from a standard joystick on the `/joy` topic.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    /// @brief Publisher for the custom `TeleopCmd` message on the `/teleop_cmd` topic.
    rclcpp::Publisher<extender_msgs::msg::TeleopCommand>::SharedPtr teleop_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr snake_publisher_;

    // Button state handling variables to detect single presses
    int last_button_1_{0}; ///< Stores the previous state of button 1 (mode toggle).
    int cur_button_1_{0};  ///< Stores the current state of button 1.
  };
} // namespace input_interfaces
