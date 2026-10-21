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

#include "hand_joystick_interface/parameter_descriptors.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

#include <rcl_interfaces/msg/integer_range.hpp>

namespace hand_joystick_interface
{
namespace
{

struct ParameterSpec
{
  std::string_view name;
  std::string_view description;
  std::string_view constraints;
  std::optional<std::array<std::int64_t, 2>> integer_range;
};

constexpr std::array<ParameterSpec, 14> kParameterSpecs{{
  {
    "dead_zone",
    "Inner control dead-zone: normalized image units for RGB or meters for OAK.",
    "Must be finite and greater than or equal to 0.",
    std::nullopt,
  },
  {
    "saturation_zone",
    "Outer control saturation boundary: normalized image units for RGB or meters for OAK.",
    "Must be finite and greater than 0.",
    std::nullopt,
  },
  {
    "normalization_mode",
    "Geometry used to normalize reference-relative landmark displacement.",
    "One of: 'axis', 'vector'.",
    std::nullopt,
  },
  {
    "lost_hand_cooldown_sec",
    "Time without a valid 21-point hand before publishing one zero command, in seconds.",
    "Must be finite and greater than or equal to 0 s.",
    std::nullopt,
  },
  {
    "landmark_index",
    "MediaPipe landmark index mapped to the normalized joystick axes.",
    "",
    std::array<std::int64_t, 2>{0, 20},
  },
  {
    "output_round_decimals",
    "Decimal places retained in each normalized command component in landmark XYZ order.",
    "",
    std::array<std::int64_t, 2>{0, 9},
  },
  {
    "enable_marker_visualization",
    "Publish an RViz arrow representing the mapped Cartesian translation command.",
    "",
    std::nullopt,
  },
  {
    "joy_output_topic",
    "Output sensor_msgs/Joy topic with normalized landmark XYZ axes.",
    "",
    std::nullopt,
  },
  {
    "cartesian_command_topic",
    "Mapped TwistStamped topic used for command visualization.",
    "Used only when marker visualization is enabled.",
    std::nullopt,
  },
  {
    "cartesian_command_marker_topic",
    "Output visualization_msgs/Marker topic.",
    "Used only when marker visualization is enabled.",
    std::nullopt,
  },
  {
    "cartesian_command_marker_frame_id",
    "Coordinate frame assigned to command visualization markers.",
    "Used only when marker visualization is enabled.",
    std::nullopt,
  },
  {
    "cartesian_command_marker_scale_x",
    "RViz arrow shaft diameter in meters.",
    "Must be finite and greater than or equal to 0 m.",
    std::nullopt,
  },
  {
    "cartesian_command_marker_scale_y",
    "RViz arrow head diameter in meters.",
    "Must be finite and greater than or equal to 0 m.",
    std::nullopt,
  },
  {
    "cartesian_command_marker_scale_z",
    "RViz arrow head length in meters.",
    "Must be finite and greater than or equal to 0 m.",
    std::nullopt,
  },
}};

}  // namespace

rcl_interfaces::msg::ParameterDescriptor parameterDescriptor(
  std::string_view parameter_name)
{
  for (const auto & spec : kParameterSpecs) {
    if (spec.name != parameter_name) {
      continue;
    }

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = spec.description;
    descriptor.additional_constraints = spec.constraints;
    if (spec.integer_range.has_value()) {
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = (*spec.integer_range)[0];
      range.to_value = (*spec.integer_range)[1];
      range.step = 1;
      descriptor.integer_range = {range};
    }
    return descriptor;
  }

  throw std::invalid_argument(
          "No descriptor registered for hand joystick parameter '" +
          std::string(parameter_name) + "'");
}

}  // namespace hand_joystick_interface
