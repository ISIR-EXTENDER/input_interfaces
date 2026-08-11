#pragma once

#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

namespace joystick_mapper
{
  enum class ButtonActivationMode
  {
    TRIGGER,
    TOGGLE,
    HOLD
  };

  class JoystickMapper : public rclcpp::Node
  {
  public:
    explicit JoystickMapper(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    struct AxisBinding
    {
      int index{-1};
      double scale{1.0};
    };

    struct AxisMap
    {
      AxisBinding linear_x;
      AxisBinding linear_y;
      AxisBinding linear_z;
      AxisBinding angular_x;
      AxisBinding angular_y;
      AxisBinding angular_z;
    };

    struct Button
    {
      int button_index{-1};
      bool previous_button_pressed{false};
      bool active{false};
      ButtonActivationMode activation_mode{ButtonActivationMode::TRIGGER};
    };

    void readParameters();

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

    double mappedAxis(const sensor_msgs::msg::Joy &msg, const AxisBinding &binding) const;
    static double axis(const sensor_msgs::msg::Joy &msg, int index);
    static bool isButtonPressed(const sensor_msgs::msg::Joy &msg, int index);
    AxisBinding declareAxisBinding(const std::string &parameter_prefix, int default_index,
                                   double default_scale = 1.0);
    AxisMap declareAxisMap(const std::string &parameter_prefix, const AxisMap &defaults);
    Button declareButton(const std::string &parameter_name, int default_button_index,
                         ButtonActivationMode default_activation_mode);
    void warnOnDuplicateButtonIndexes() const;

    void handleStateButtons(const sensor_msgs::msg::Joy &msg);
    void handleLocalModeButton(const sensor_msgs::msg::Joy &msg);
    void handleStateButton(const sensor_msgs::msg::Joy &msg, Button &button,
                           std::string &current_state, const std::string &target_state,
                           const std::string &default_state, const std::string &request_scope);
    void handleCommandButton(const sensor_msgs::msg::Joy &msg, Button &button,
                             const std::string &request, const std::string &release_request = {});
    void publishModeRequest(const std::string &request);

    std::string joy_topic_;
    std::string output_topic_;
    std::string mode_request_topic_;
    std::string output_frame_id_{"base_link"};

    double deadzone_{0.2};
    AxisMap default_axes_{{0, 1.0}, {1, 1.0}, {2, 1.0}, {-1, 1.0}, {-1, 1.0}, {-1, 1.0}};
    AxisMap b2_axes_;
    const AxisMap *active_axes_{&default_axes_};

    Button local_mode_button_;
    Button jaco_button_;
    Button snake_button_;
    Button home_button_;

    std::string current_geometric_state_{"both"};

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_request_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  };
} // namespace joystick_mapper
