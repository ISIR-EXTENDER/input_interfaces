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

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>
#include <string_view>

#include "hand_joystick_interface/parameter_descriptors.hpp"

namespace hand_joystick_interface
{
namespace
{

constexpr std::array<std::string_view, 14> kParameterNames{{
  "dead_zone",
  "saturation_zone",
  "normalization_mode",
  "lost_hand_cooldown_sec",
  "landmark_index",
  "output_round_decimals",
  "enable_marker_visualization",
  "joy_output_topic",
  "cartesian_command_topic",
  "teleop_cmd_marker_topic",
  "teleop_cmd_marker_frame_id",
  "teleop_cmd_marker_scale_x",
  "teleop_cmd_marker_scale_y",
  "teleop_cmd_marker_scale_z",
}};

TEST(ParameterDescriptors, CoversEveryPublicParameter)
{
  for (const auto parameter_name : kParameterNames) {
    const auto descriptor = parameterDescriptor(parameter_name);
    EXPECT_FALSE(descriptor.description.empty()) << parameter_name;
  }
}

TEST(ParameterDescriptors, RejectsUnknownParameters)
{
  EXPECT_THROW(parameterDescriptor("unknown_parameter"), std::invalid_argument);
}

TEST(ParameterDescriptors, ListsNormalizationModes)
{
  const auto descriptor = parameterDescriptor("normalization_mode");
  EXPECT_NE(descriptor.additional_constraints.find("'axis', 'vector'"),
    std::string::npos);
}

TEST(ParameterDescriptors, DescribesLandmarkIndexRange)
{
  const auto descriptor = parameterDescriptor("landmark_index");
  ASSERT_EQ(descriptor.integer_range.size(), 1U);
  EXPECT_EQ(descriptor.integer_range[0].from_value, 0);
  EXPECT_EQ(descriptor.integer_range[0].to_value, 20);
  EXPECT_EQ(descriptor.integer_range[0].step, 1U);
}

TEST(ParameterDescriptors, DescribesRoundingRange)
{
  const auto descriptor = parameterDescriptor("output_round_decimals");
  ASSERT_EQ(descriptor.integer_range.size(), 1U);
  EXPECT_EQ(descriptor.integer_range[0].from_value, 0);
  EXPECT_EQ(descriptor.integer_range[0].to_value, 9);
}

}  // namespace
}  // namespace hand_joystick_interface
