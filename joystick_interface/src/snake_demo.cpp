#include "joystick_interface/snake_demo.hpp"

namespace input_interfaces
{
  JoystickSnake::JoystickSnake() : Node("snake_joystick")
  {
    typedef extender_msgs::msg::TeleopCommand Mode;
    // Create a subscription to the 3D joystick Joy messages on "/joy".
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&JoystickSnake::joyCallback, this, std::placeholders::_1));

    // Publisher for custom msg : Twist + teleop mode + gripper command
    teleop_cmd_publisher_ =
        this->create_publisher<extender_msgs::msg::TeleopCommand>("/teleop_cmd", 10);

    snake_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/activate_snake", 10);

    RCLCPP_INFO(this->get_logger(), "Joystick Controller node initialized");
  }

  // Callback for /joy (3D joystick)
  void JoystickSnake::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    typedef extender_msgs::msg::TeleopCommand Mode;

    extender_msgs::msg::TeleopCommand cmd_msg;

    // Update current button states from message (joystick mode button)
    cur_button_1_ = msg->buttons[11];

    // --- Mode switch: rising edge (single event per press) ---
    if (cur_button_1_ == 1 && last_button_1_ == 0)
    {
      current_mode_ =
          (current_mode_ == Mode::TRANSLATION_ROTATION) ? Mode::BOTH : Mode::TRANSLATION_ROTATION;
    }

    last_button_1_ = cur_button_1_;
    auto snake_msg = std_msgs::msg::Bool();

    if (msg->buttons[10] == 1)
    {
      snake_msg.data = true;
    }
    else
    {
      snake_msg.data = false;
    }
    snake_publisher_->publish(snake_msg);

    auto twist = geometry_msgs::msg::Twist();
    if (!msg->axes.empty())
    {
      // Helper: read an axis from the Joy message
      auto getAxisValue = [&msg](int axis_index) -> double {
        if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg->axes.size())
        {
          return 0.0;
        }
        return msg->axes[axis_index];
      };

      twist.linear.x = getAxisValue(0);
      twist.linear.y = getAxisValue(1);
    }

    cmd_msg.twist = twist;
    cmd_msg.mode = static_cast<uint8_t>(current_mode_);
    teleop_cmd_publisher_->publish(cmd_msg);
  }
} // namespace input_interfaces
