#include "joystick_mapper/joystick_mapper.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "signal_processing/dead_zone.hpp"

namespace joystick_mapper
{
  namespace
  {
    constexpr char kGeometricScope[] = "geometric";
    constexpr char kGeometricBoth[] = "both";
    constexpr char kGeometricJaco[] = "jaco";
    constexpr char kGeometricSnake[] = "snake";
    constexpr char kHomeRequest[] = "behaviour/joint_target/home";
    constexpr char kPassthroughRequest[] = "behaviour/passthrough";

    std::string normalizeStateName(std::string state)
    {
      std::transform(state.begin(), state.end(), state.begin(),
                     [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
      std::replace(state.begin(), state.end(), '-', '_');
      return state;
    }

    std::string buttonModeParameterName(const std::string &button_index_parameter_name)
    {
      constexpr char suffix[] = "_index";
      constexpr auto suffix_length = std::char_traits<char>::length(suffix);
      if (button_index_parameter_name.size() >= suffix_length &&
          button_index_parameter_name.compare(button_index_parameter_name.size() - suffix_length,
                                              suffix_length, suffix) == 0)
      {
        return button_index_parameter_name.substr(0,
                                                  button_index_parameter_name.size() - suffix_length) +
               "_mode";
      }

      return button_index_parameter_name + "_mode";
    }

    std::string activationModeName(ButtonActivationMode mode)
    {
      switch (mode)
      {
      case ButtonActivationMode::TRIGGER:
        return "trigger";
      case ButtonActivationMode::TOGGLE:
        return "toggle";
      case ButtonActivationMode::HOLD:
        return "hold";
      }

      return "trigger";
    }

    ButtonActivationMode activationModeFromName(const std::string &raw_mode,
                                               ButtonActivationMode fallback)
    {
      const auto mode = normalizeStateName(raw_mode);
      if (mode == "trigger")
      {
        return ButtonActivationMode::TRIGGER;
      }
      if (mode == "toggle")
      {
        return ButtonActivationMode::TOGGLE;
      }
      if (mode == "hold" || mode == "momentary" || mode == "pressed")
      {
        return ButtonActivationMode::HOLD;
      }

      return fallback;
    }

  } // namespace

  JoystickMapper::JoystickMapper(const rclcpp::NodeOptions &options)
      : rclcpp::Node("joystick_mapper", options)
  {
    readParameters();

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
        joy_topic_, 10, std::bind(&JoystickMapper::joyCallback, this, std::placeholders::_1));

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
    mode_request_pub_ = create_publisher<std_msgs::msg::String>(mode_request_topic_, 10);
  }

  void JoystickMapper::readParameters()
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    output_topic_ = declare_parameter<std::string>("output_topic", "/joystick_cartesian_command");
    mode_request_topic_ = declare_parameter<std::string>("mode_request_topic", "/mode_request");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "base_link");

    deadzone_ = declare_parameter<double>("deadzone", 0.2);
    const AxisMap default_axes{{0, 1.0}, {1, 1.0}, {2, 1.0}, {-1, 1.0}, {-1, 1.0}, {-1, 1.0}};
    const AxisMap b2_default_axes{{-1, 1.0}, {-1, 1.0}, {-1, 1.0}, {0, 1.0}, {1, 1.0}, {2, 1.0}};

    default_axes_ = declareAxisMap("axes", default_axes);
    b2_axes_ = declareAxisMap("modes.b2.axes", b2_default_axes);

    local_mode_button_ =
        declareButton("local_mode_button_index", -1, ButtonActivationMode::TOGGLE);
    jaco_button_ = declareButton("jaco_button_index", -1, ButtonActivationMode::TOGGLE);
    snake_button_ = declareButton("snake_button_index", -1, ButtonActivationMode::TOGGLE);
    home_button_ = declareButton("home_button_index", -1, ButtonActivationMode::TRIGGER);
    warnOnDuplicateButtonIndexes();

    if (deadzone_ < 0.0 || deadzone_ >= 1.0)
    {
      RCLCPP_WARN(get_logger(), "Invalid deadzone %.3f, using 0.2", deadzone_);
      deadzone_ = 0.2;
    }
  }

  JoystickMapper::AxisBinding
  JoystickMapper::declareAxisBinding(const std::string &parameter_prefix, int default_index,
                                     double default_scale)
  {
    AxisBinding binding;
    binding.index = declare_parameter<int>(parameter_prefix + ".index", default_index);
    binding.scale = declare_parameter<double>(parameter_prefix + ".scale", default_scale);

    if (binding.index < -1)
    {
      RCLCPP_WARN(get_logger(), "Invalid axis index %d for %s, disabling this axis", binding.index,
                  parameter_prefix.c_str());
      binding.index = -1;
    }

    if (!std::isfinite(binding.scale))
    {
      RCLCPP_WARN(get_logger(), "Invalid axis scale for %s, using 1.0", parameter_prefix.c_str());
      binding.scale = 1.0;
    }

    return binding;
  }

  JoystickMapper::AxisMap JoystickMapper::declareAxisMap(const std::string &parameter_prefix,
                                                         const AxisMap &defaults)
  {
    AxisMap axes;
    axes.linear_x = declareAxisBinding(parameter_prefix + ".linear_x", defaults.linear_x.index,
                                       defaults.linear_x.scale);
    axes.linear_y = declareAxisBinding(parameter_prefix + ".linear_y", defaults.linear_y.index,
                                       defaults.linear_y.scale);
    axes.linear_z = declareAxisBinding(parameter_prefix + ".linear_z", defaults.linear_z.index,
                                       defaults.linear_z.scale);
    axes.angular_x = declareAxisBinding(parameter_prefix + ".angular_x", defaults.angular_x.index,
                                        defaults.angular_x.scale);
    axes.angular_y = declareAxisBinding(parameter_prefix + ".angular_y", defaults.angular_y.index,
                                        defaults.angular_y.scale);
    axes.angular_z = declareAxisBinding(parameter_prefix + ".angular_z", defaults.angular_z.index,
                                        defaults.angular_z.scale);
    return axes;
  }

  JoystickMapper::Button JoystickMapper::declareButton(
      const std::string &parameter_name, int default_button_index,
      ButtonActivationMode default_activation_mode)
  {
    Button button;
    button.button_index = declare_parameter<int>(parameter_name, default_button_index);
    const auto mode_parameter_name = buttonModeParameterName(parameter_name);
    const auto default_mode_name = activationModeName(default_activation_mode);
    const auto requested_mode =
        declare_parameter<std::string>(mode_parameter_name, default_mode_name);
    button.activation_mode = activationModeFromName(requested_mode, default_activation_mode);

    if (button.button_index < -1)
    {
      RCLCPP_WARN(get_logger(), "Invalid button index %d for %s, disabling this button",
                  button.button_index, parameter_name.c_str());
      button.button_index = -1;
    }

    const auto normalized_mode = normalizeStateName(requested_mode);
    if (normalized_mode != "trigger" && normalized_mode != "toggle" && normalized_mode != "hold" &&
        normalized_mode != "momentary" && normalized_mode != "pressed")
    {
      RCLCPP_WARN(get_logger(), "Invalid button mode '%s' for %s, using %s",
                  requested_mode.c_str(), mode_parameter_name.c_str(),
                  activationModeName(button.activation_mode).c_str());
    }

    return button;
  }

  void JoystickMapper::warnOnDuplicateButtonIndexes() const
  {
    const std::vector<std::pair<std::string, int>> buttons{
        {"local_mode_button_index", local_mode_button_.button_index},
        {"jaco_button_index", jaco_button_.button_index},
        {"snake_button_index", snake_button_.button_index},
        {"home_button_index", home_button_.button_index},
    };

    for (std::size_t i = 0; i < buttons.size(); ++i)
    {
      if (buttons[i].second < 0)
      {
        continue;
      }

      for (std::size_t j = i + 1; j < buttons.size(); ++j)
      {
        if (buttons[i].second == buttons[j].second)
        {
          RCLCPP_WARN(get_logger(),
                      "Duplicate joystick button index %d for %s and %s; one button press will "
                      "trigger both actions",
                      buttons[i].second, buttons[i].first.c_str(), buttons[j].first.c_str());
        }
      }
    }
  }

  void JoystickMapper::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    handleStateButtons(*msg);

    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = now();
    output.header.frame_id = output_frame_id_;

    const auto &axes = *active_axes_;
    output.twist.linear.x = mappedAxis(*msg, axes.linear_x);
    output.twist.linear.y = mappedAxis(*msg, axes.linear_y);
    output.twist.linear.z = mappedAxis(*msg, axes.linear_z);
    output.twist.angular.x = mappedAxis(*msg, axes.angular_x);
    output.twist.angular.y = mappedAxis(*msg, axes.angular_y);
    output.twist.angular.z = mappedAxis(*msg, axes.angular_z);

    twist_pub_->publish(output);
  }

  double JoystickMapper::mappedAxis(const sensor_msgs::msg::Joy &msg,
                                    const AxisBinding &binding) const
  {
    return binding.scale *
           signal_processing::applyScaledDeadZone(axis(msg, binding.index), deadzone_, 1.0);
  }

  double JoystickMapper::axis(const sensor_msgs::msg::Joy &msg, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < msg.axes.size()
               ? static_cast<double>(msg.axes[static_cast<std::size_t>(index)])
               : 0.0;
  }

  bool JoystickMapper::isButtonPressed(const sensor_msgs::msg::Joy &msg, int index)
  {
    return index >= 0 && static_cast<std::size_t>(index) < msg.buttons.size() &&
           msg.buttons[static_cast<std::size_t>(index)] != 0;
  }

  void JoystickMapper::handleStateButtons(const sensor_msgs::msg::Joy &msg)
  {
    handleLocalModeButton(msg);
    handleStateButton(msg, jaco_button_, current_geometric_state_, kGeometricJaco,
                      kGeometricBoth, kGeometricScope);
    handleStateButton(msg, snake_button_, current_geometric_state_, kGeometricSnake,
                      kGeometricBoth, kGeometricScope);
    handleCommandButton(msg, home_button_, kHomeRequest, kPassthroughRequest);
  }

  void JoystickMapper::handleLocalModeButton(const sensor_msgs::msg::Joy &msg)
  {
    const bool pressed = isButtonPressed(msg, local_mode_button_.button_index);
    if (local_mode_button_.activation_mode == ButtonActivationMode::HOLD)
    {
      active_axes_ = pressed ? &b2_axes_ : &default_axes_;
    }
    else if (pressed && !local_mode_button_.previous_button_pressed)
    {
      active_axes_ = active_axes_ == &b2_axes_ ? &default_axes_ : &b2_axes_;
    }
    local_mode_button_.previous_button_pressed = pressed;
  }

  void JoystickMapper::handleStateButton(
      const sensor_msgs::msg::Joy &msg, Button &button, std::string &current_state,
      const std::string &target_state, const std::string &default_state,
      const std::string &request_scope)
  {
    const bool pressed = isButtonPressed(msg, button.button_index);
    if (button.activation_mode == ButtonActivationMode::HOLD)
    {
      if (pressed && !button.previous_button_pressed)
      {
        current_state = target_state;
        publishModeRequest(request_scope + "/" + current_state);
      }
      else if (!pressed && button.previous_button_pressed && current_state == target_state)
      {
        current_state = default_state;
        publishModeRequest(request_scope + "/" + current_state);
      }
    }
    else if (pressed && !button.previous_button_pressed)
    {
      if (button.activation_mode == ButtonActivationMode::TRIGGER)
      {
        current_state = target_state;
      }
      else
      {
        current_state = current_state == target_state ? default_state : target_state;
      }
      publishModeRequest(request_scope + "/" + current_state);
    }
    button.previous_button_pressed = pressed;
  }

  void JoystickMapper::handleCommandButton(const sensor_msgs::msg::Joy &msg, Button &button,
                                           const std::string &request,
                                           const std::string &release_request)
  {
    const bool pressed = isButtonPressed(msg, button.button_index);
    if (button.activation_mode == ButtonActivationMode::HOLD)
    {
      if (pressed && !button.previous_button_pressed)
      {
        button.active = true;
        publishModeRequest(request);
      }
      else if (!pressed && button.previous_button_pressed)
      {
        button.active = false;
        if (!release_request.empty())
        {
          publishModeRequest(release_request);
        }
      }
    }
    else if (pressed && !button.previous_button_pressed)
    {
      if (button.activation_mode == ButtonActivationMode::TOGGLE)
      {
        button.active = !button.active;
        if (!button.active && !release_request.empty())
        {
          publishModeRequest(release_request);
        }
        else
        {
          publishModeRequest(request);
        }
      }
      else
      {
        publishModeRequest(request);
      }
    }
    button.previous_button_pressed = pressed;
  }

  void JoystickMapper::publishModeRequest(const std::string &request)
  {
    std_msgs::msg::String msg;
    msg.data = normalizeStateName(request);
    mode_request_pub_->publish(msg);
  }
} // namespace joystick_mapper
