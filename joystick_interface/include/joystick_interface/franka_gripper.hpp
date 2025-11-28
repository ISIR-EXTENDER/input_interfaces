#pragma once

#include "franka_msgs/action/grasp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace input_interfaces
{
  class FrankaGripper : public rclcpp::Node
  {
  public:
    FrankaGripper();

    /// @brief Callback function to process incoming Joy messages from the SpaceMouse.
    void spaceMouseCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    /// @brief Callback function to process incoming Joy messages from the 3D joystick.
    void joy3dCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

  private:
    /// @brief Subscriber for receiving Joy messages from the SpaceMouse on the `/spacenav/joy`
    /// topic.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr spacenav_joy_subscriber_;

    /// @brief Subscriber for receiving Joy messages from a standard joystick on the `/joy` topic.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    rclcpp_action::Client<franka_msgs::action::Grasp>::SharedPtr gripper_action_client_;

    // Gripper parameters.
    double open_width_;
    double closed_width_;
    double closing_speed_;
    double closing_force_;

    /// @brief state of the gripper
    bool is_gripper_closed_{false};
    int last_button_gripper_{0}; ///< Stores the previous state of button 0 (gripper).
    int cur_button_gripper_{0};  ///< Stores the current state of button 0.
  };
} // namespace input_interfaces