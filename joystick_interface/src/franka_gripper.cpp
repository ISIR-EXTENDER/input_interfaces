#include "joystick_interface/franka_gripper.hpp"

namespace input_interfaces
{

  FrankaGripper::FrankaGripper() : Node("franka_gripper")
  {
    using namespace std::chrono_literals;

    // Read gripper parameters from config file
    // Using unique parameter names to avoid conflicts
    if (!this->has_parameter("gripper_closed_width"))
    {
      this->declare_parameter("gripper_closed_width", 0.00);
    }
    if (!this->has_parameter("gripper_open_width"))
    {
      this->declare_parameter("gripper_open_width", 0.02);
    }
    if (!this->has_parameter("gripper_closing_speed"))
    {
      this->declare_parameter("gripper_closing_speed", 0.03);
    }
    if (!this->has_parameter("gripper_closing_force"))
    {
      this->declare_parameter("gripper_closing_force", 50.0);
    }
    open_width_ = this->get_parameter("gripper_open_width").as_double();
    closed_width_ = this->get_parameter("gripper_closed_width").as_double();
    closing_speed_ = this->get_parameter("gripper_closing_speed").as_double();
    closing_force_ = this->get_parameter("gripper_closing_force").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Gripper parameters: closed_width: %.2f, closing_speed: %.2f, closing_force: %.2f",
                closed_width_, closing_speed_, closing_force_);

    // Create a subscription to the SpaceMouse Joy messages on "/spacenav/joy".
    spacenav_joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/spacenav/joy", 10, std::bind(&FrankaGripper::spaceMouseCallback, this, std::placeholders::_1));

    // Create a subscription to the 3D joystick Joy messages on "/joy".
    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&FrankaGripper::joy3dCallback, this, std::placeholders::_1));

    gripper_action_client_ =
        rclcpp_action::create_client<franka_msgs::action::Grasp>(this, "/fr3_gripper/grasp");
    if (!gripper_action_client_->wait_for_action_server(5s)) {
          RCLCPP_ERROR(this->get_logger(), "Grasping action server not available.");
        }
      
    RCLCPP_INFO(this->get_logger(), "Franka gripper connected. Waiting for teleop commands...");
  }

  void FrankaGripper::spaceMouseCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Guard against short button arrays (e.g. different devices or misconfigured drivers).
    if (!msg->buttons.empty())
    {
      cur_button_gripper_ = msg->buttons[0];
    }
    else
    {
      cur_button_gripper_ = 0;
    }
    auto goal_msg = franka_msgs::action::Grasp::Goal();
    goal_msg.speed = closing_speed_;
    goal_msg.force = closing_force_;
    // --- Gripper command ---
    if (cur_button_gripper_ == 1 && last_button_gripper_ == 0)
    {
      if (!is_gripper_closed_)
      {
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_CLOSE");
        goal_msg.width = closed_width_;
        is_gripper_closed_ = true;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_OPEN");
        goal_msg.width = open_width_;
        is_gripper_closed_ = false;
      }
    
      if(gripper_action_client_->action_server_is_ready())
        gripper_action_client_->async_send_goal(goal_msg);
      else
        RCLCPP_WARN(this->get_logger(), "Gripper action server not ready");
    }
    last_button_gripper_ = cur_button_gripper_;
  }

  void FrankaGripper::joy3dCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons.size() > 10)
    {
      cur_button_gripper_ = msg->buttons[10];
    }
    else
    {
      cur_button_gripper_ = 0;
    }
    auto goal_msg = franka_msgs::action::Grasp::Goal();
    goal_msg.speed = closing_speed_;
    goal_msg.force = closing_force_;
    // --- Gripper command ---
    if (cur_button_gripper_ == 1 && last_button_gripper_ == 0)
    {
      if (!is_gripper_closed_)
      {
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_CLOSE");
        goal_msg.width = closed_width_;
        is_gripper_closed_ = true;
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Button 0: Sending GRIPPER_CMD_OPEN");
        goal_msg.width = open_width_;
        is_gripper_closed_ = false;
      }
    
      if(gripper_action_client_->action_server_is_ready())
        gripper_action_client_->async_send_goal(goal_msg);
      else
        RCLCPP_WARN(this->get_logger(), "Gripper action server not ready");
    }
    last_button_gripper_ = cur_button_gripper_;
  }
} // namespace input_interfaces

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<input_interfaces::FrankaGripper>());
  rclcpp::shutdown();
  return 0;
}