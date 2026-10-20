// Copyright 2026 ISIR-EXTENDER
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include "hand_joystick_interface/hand_joystick_interface.hpp"

#include <Eigen/Core>

#include <chrono>
#include <cmath>

#include "hand_joystick_interface/parameter_descriptors.hpp"

namespace hand_joystick_interface
{

HandJoystickInterface::HandJoystickInterface()
: Node("hand_joystick_interface"),
  last_valid_hand_time_sec_(-1.0),
  lost_hand_zero_sent_(false)
{
  // Declare parameters
  this->declare_parameter(
    "dead_zone", 0.05, parameterDescriptor("dead_zone"));
  this->declare_parameter(
    "saturation_zone", 0.2, parameterDescriptor("saturation_zone"));
  this->declare_parameter<std::string>(
    "normalization_mode", "vector", parameterDescriptor("normalization_mode"));
  this->declare_parameter(
    "lost_hand_cooldown_sec",
    0.2,
    parameterDescriptor("lost_hand_cooldown_sec"));
  this->declare_parameter(
    "landmark_index", 8, parameterDescriptor("landmark_index"));
  this->declare_parameter(
    "output_round_decimals",
    2,
    parameterDescriptor("output_round_decimals"));
  this->declare_parameter(
    "enable_marker_visualization",
    true,
    parameterDescriptor("enable_marker_visualization"));
  this->declare_parameter(
    "joy_output_topic",
    "/hand_joy",
    parameterDescriptor("joy_output_topic"));
  this->declare_parameter(
    "cartesian_command_topic",
    "/joystick_cartesian_command",
    parameterDescriptor("cartesian_command_topic"));
  this->declare_parameter(
    "cartesian_command_marker_topic",
    "cartesian_command_marker",
    parameterDescriptor("cartesian_command_marker_topic"));
  this->declare_parameter(
    "cartesian_command_marker_frame_id",
    "base_link",
    parameterDescriptor("cartesian_command_marker_frame_id"));
  this->declare_parameter(
    "cartesian_command_marker_scale_x",
    0.03,
    parameterDescriptor("cartesian_command_marker_scale_x"));
  this->declare_parameter(
    "cartesian_command_marker_scale_y",
    0.07,
    parameterDescriptor("cartesian_command_marker_scale_y"));
  this->declare_parameter(
    "cartesian_command_marker_scale_z",
    0.07,
    parameterDescriptor("cartesian_command_marker_scale_z"));
  // Get parameters
  control_mapping_config_.dead_zone =
    this->get_parameter("dead_zone").as_double();
  control_mapping_config_.saturation_zone =
    this->get_parameter("saturation_zone").as_double();
  normalization_mode_name_ = this->get_parameter("normalization_mode").as_string();
  lost_hand_cooldown_sec_ = this->get_parameter("lost_hand_cooldown_sec").as_double();
  landmark_index_ = static_cast<std::size_t>(
    this->get_parameter("landmark_index").as_int());
  control_mapping_config_.output_round_decimals = static_cast<int>(
    this->get_parameter("output_round_decimals").as_int());
  enable_marker_visualization_ = this->get_parameter("enable_marker_visualization").as_bool();
  joy_output_topic_ = this->get_parameter("joy_output_topic").as_string();
  const auto marker_topic = this->get_parameter("cartesian_command_marker_topic").as_string();
  marker_frame_id_ = this->get_parameter("cartesian_command_marker_frame_id").as_string();
  marker_scale_x_ = this->get_parameter("cartesian_command_marker_scale_x").as_double();
  marker_scale_y_ = this->get_parameter("cartesian_command_marker_scale_y").as_double();
  marker_scale_z_ = this->get_parameter("cartesian_command_marker_scale_z").as_double();
  if (!std::isfinite(control_mapping_config_.dead_zone) ||
    control_mapping_config_.dead_zone < 0.0)
  {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'dead_zone' must be finite and >= 0. Falling back to 0.0");
    control_mapping_config_.dead_zone = 0.0;
  }
  if (!std::isfinite(control_mapping_config_.saturation_zone) ||
    control_mapping_config_.saturation_zone <= 0.0)
  {
    RCLCPP_WARN(this->get_logger(),
        "Parameter 'saturation_zone' must be finite and > 0. Falling back to 1e-6");
    control_mapping_config_.saturation_zone = 1e-6;
  }

  const auto normalization_mode =
    parseNormalizationMode(normalization_mode_name_);
  if (normalization_mode.has_value()) {
    control_mapping_config_.normalization_mode = *normalization_mode;
  } else {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'normalization_mode' must be 'axis' or 'vector'. Falling back to 'vector'");
    control_mapping_config_.normalization_mode = NormalizationMode::Vector;
    normalization_mode_name_ = "vector";
  }

  if (!std::isfinite(lost_hand_cooldown_sec_) ||
    lost_hand_cooldown_sec_ < 0.0)
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Parameter 'lost_hand_cooldown_sec' must be finite and >= 0. Falling back to 0.2");
    lost_hand_cooldown_sec_ = 0.2;
  }

  // Create subscription to hand_landmarks (PointCloud)
  // Handles both 2D (z=0) and 3D (z≠0) automatically
  hand_landmarks_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud>(
    "hand_landmarks",
    10,
    std::bind(&HandJoystickInterface::handLandmarksCallback, this, std::placeholders::_1)
  );

  hand_loss_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&HandJoystickInterface::checkHandLossTimeout, this));

  // Create publishers
  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_output_topic_, 10);

  if (enable_marker_visualization_) {
    cartesian_command_marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
    cartesian_command_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->get_parameter("cartesian_command_topic").as_string(), 10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr command) {
        publishMarker(*command);
      });
  }

  RCLCPP_INFO(this->get_logger(), "Hand Joystick Interface initialized");
  RCLCPP_INFO(
    this->get_logger(),
    "Dead zone: %.2f",
    control_mapping_config_.dead_zone);
  RCLCPP_INFO(
    this->get_logger(),
    "XYZ Saturation zone: %.2f",
    control_mapping_config_.saturation_zone);
  RCLCPP_INFO(this->get_logger(), "Normalization mode: %s", normalization_mode_name_.c_str());
  RCLCPP_INFO(this->get_logger(), "Tracking landmark index: %zu", landmark_index_);
  RCLCPP_INFO(
    this->get_logger(),
    "Output rounding decimals: %d",
    control_mapping_config_.output_round_decimals);
  RCLCPP_INFO(this->get_logger(), "Joy output topic: %s", joy_output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Marker visualization: %s",
      enable_marker_visualization_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "Lost-hand cooldown: %.3f s", lost_hand_cooldown_sec_);
  if (enable_marker_visualization_) {
    RCLCPP_INFO(this->get_logger(), "Cartesian command marker topic: %s (frame_id: %s)",
        marker_topic.c_str(), marker_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Cartesian command marker scale: [%.3f, %.3f, %.3f]",
        marker_scale_x_,
        marker_scale_y_, marker_scale_z_);
  }
  RCLCPP_INFO(this->get_logger(),
      "Note: hand landmark coordinates are expected to be relative to their reference");
}

void HandJoystickInterface::publishZeroCommand()
{
  publishJoy(Eigen::Vector3d::Zero());
}

void HandJoystickInterface::publishJoy(const Eigen::Vector3d & axes)
{
  sensor_msgs::msg::Joy joy;
  joy.header.stamp = this->now();
  joy.axes = {static_cast<float>(axes.x()), static_cast<float>(axes.y()),
    static_cast<float>(axes.z())};
  joy_pub_->publish(joy);
}

void HandJoystickInterface::checkHandLossTimeout()
{
  if (last_valid_hand_time_sec_ < 0.0 || lost_hand_zero_sent_) {
    return;
  }

  const double now_sec = this->now().seconds();
  if ((now_sec - last_valid_hand_time_sec_) < lost_hand_cooldown_sec_) {
    return;
  }

  publishZeroCommand();
  lost_hand_zero_sent_ = true;

  RCLCPP_WARN(this->get_logger(),
              "Hand landmarks lost for %.3f s; publishing zero",
              lost_hand_cooldown_sec_);
}

void HandJoystickInterface::handLandmarksCallback(
  const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
  const size_t num_points = msg->points.size();
  if (num_points != kHandLandmarkCount) {
    RCLCPP_WARN(this->get_logger(),
        "Received malformed hand landmark cloud (got %zu points, expected %zu)",
        num_points,
        kHandLandmarkCount);
    return;
  }

  // Extract reference-relative x, y, z coordinates from the specified landmark.
  const auto & landmark = msg->points[landmark_index_];
  const double x = static_cast<double>(landmark.x);
  const double y = static_cast<double>(landmark.y);
  const double z = static_cast<double>(landmark.z);
  const Eigen::Vector3d input(x, y, z);
  if (!isFiniteLandmark(input)) {
    RCLCPP_WARN(this->get_logger(),
        "Received non-finite hand landmark at index %zu; ignoring cloud",
        landmark_index_);
    return;
  }

  last_valid_hand_time_sec_ = this->now().seconds();
  lost_hand_zero_sent_ = false;

  // Check dead zone
  if (isInDeadZone(x, y, z)) {
    // Publish zero velocity in dead zone
    publishZeroCommand();
    RCLCPP_DEBUG(this->get_logger(), "Hand in dead zone; publishing zero command");
    return;
  }

  // Normalize in landmark XYZ order; the mapper selects robot axes and signs.
  const Eigen::Vector3d axes = mapLandmarkToCommand(input, control_mapping_config_);
  publishJoy(axes);
  RCLCPP_DEBUG(this->get_logger(),
    "Relative hand: (%.3f, %.3f, %.3f) -> Joy: (%.2f, %.2f, %.2f)",
    x, y, z, axes.x(), axes.y(), axes.z());
}

bool HandJoystickInterface::isInDeadZone(double x, double y, double z)
{
  const Eigen::Vector3d input_vector(x, y, z);
  if (control_mapping_config_.normalization_mode == NormalizationMode::Axis) {
    const bool all_axes_in_dead_zone =
      hand_joystick_interface::isInDeadZone(
      input_vector,
      control_mapping_config_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Hand axis dead-zone check: [%d, %d, %d] (dead zone: %.3f)",
      std::abs(x) <= control_mapping_config_.dead_zone,
      std::abs(y) <= control_mapping_config_.dead_zone,
      std::abs(z) <= control_mapping_config_.dead_zone,
      control_mapping_config_.dead_zone);
    return all_axes_in_dead_zone;
  }

  const double distance = input_vector.norm();
  RCLCPP_DEBUG(
    this->get_logger(),
    "Hand vector distance from reference: %.3f (dead zone: %.3f)",
    distance,
    control_mapping_config_.dead_zone);
  return hand_joystick_interface::isInDeadZone(
    input_vector,
    control_mapping_config_);
}

void HandJoystickInterface::publishMarker(const geometry_msgs::msg::TwistStamped & command)
{
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header.stamp = this->now();
  marker_msg.header.frame_id = marker_frame_id_;
  marker_msg.ns = "cartesian_command";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::ARROW;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.scale.x = marker_scale_x_;
  marker_msg.scale.y = marker_scale_y_;
  marker_msg.scale.z = marker_scale_z_;
  marker_msg.color.a = 1.0;
  marker_msg.color.r = 0.0;
  marker_msg.color.g = 1.0;
  marker_msg.color.b = 0.0;

  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;
  start.x = 0.0;
  start.y = 0.0;
  start.z = 0.0;
  end.x = command.twist.linear.x;
  end.y = command.twist.linear.y;
  end.z = command.twist.linear.z;

  marker_msg.points = {start, end};
  cartesian_command_marker_pub_->publish(marker_msg);
}

}  // namespace hand_joystick_interface
