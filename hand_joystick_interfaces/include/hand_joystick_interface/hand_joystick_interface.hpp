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


#ifndef HAND_JOYSTICK_INTERFACE__HAND_JOYSTICK_INTERFACE_HPP_
#define HAND_JOYSTICK_INTERFACE__HAND_JOYSTICK_INTERFACE_HPP_

#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "hand_joystick_interface/control_mapping.hpp"

namespace hand_joystick_interface
{

class HandJoystickInterface : public rclcpp::Node
{
public:
  HandJoystickInterface();
  ~HandJoystickInterface() = default;

  // Publish an explicit zero Joy message, used during shutdown safety.
  void publishZeroCommand();

private:
  // Unified callback (handles both 2D with z=0 and 3D with z≠0)
  void handLandmarksCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

  // Watchdog that stops motion when hand landmarks stop arriving
  void checkHandLossTimeout();

  // Publish normalized landmark XYZ axes
  void publishJoy(const Eigen::Vector3d & axes);

  // Dead zone check (3D distance, works for 2D when z=0)
  bool isInDeadZone(double x, double y, double z);

  // Publish visualization marker
  void publishMarker(const geometry_msgs::msg::TwistStamped & command);

  // ROS 2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr hand_landmarks_sub_;
  rclcpp::TimerBase::SharedPtr hand_loss_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cartesian_command_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr teleop_cmd_marker_pub_;

  // Parameters
  ControlMappingConfig control_mapping_config_;
  std::string normalization_mode_name_;
  double lost_hand_cooldown_sec_;
  std::size_t landmark_index_;
  bool enable_marker_visualization_;
  std::string joy_output_topic_;
  std::string marker_frame_id_;
  double marker_scale_x_;
  double marker_scale_y_;
  double marker_scale_z_;

  // State
  double last_valid_hand_time_sec_;
  bool lost_hand_zero_sent_;
};

}  // namespace hand_joystick_interface

#endif  // HAND_JOYSTICK_INTERFACE__HAND_JOYSTICK_INTERFACE_HPP_
