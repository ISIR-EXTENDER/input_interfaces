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

#include <Eigen/Core>

#include <cstdint>
#include <limits>

#include "hand_joystick_interface/control_mapping.hpp"

namespace hand_joystick_interface
{
namespace
{

TEST(ControlMapping, ParsesSupportedNormalizationModes)
{
  EXPECT_EQ(parseNormalizationMode("axis"), NormalizationMode::Axis);
  EXPECT_EQ(parseNormalizationMode("vector"), NormalizationMode::Vector);
  EXPECT_FALSE(parseNormalizationMode("unsupported").has_value());
}

TEST(ControlMapping, ValidatesLandmarkIndexBounds)
{
  EXPECT_FALSE(isValidLandmarkIndex(-1));
  EXPECT_TRUE(isValidLandmarkIndex(0));
  EXPECT_TRUE(isValidLandmarkIndex(20));
  EXPECT_FALSE(isValidLandmarkIndex(21));
}

TEST(ControlMapping, PublishesNormalizedAxesInLandmarkOrder)
{
  ControlMappingConfig config;
  config.dead_zone = 0.05;
  config.saturation_zone = 0.25;
  config.normalization_mode = NormalizationMode::Axis;

  const Eigen::Vector3d command = mapLandmarkToCommand(
    Eigen::Vector3d(0.15, -0.30, 0.05),
    config);

  EXPECT_DOUBLE_EQ(command.x(), 0.5);
  EXPECT_DOUBLE_EQ(command.y(), -1.0);
  EXPECT_DOUBLE_EQ(command.z(), 0.0);
}

TEST(ControlMapping, PreservesDirectionInVectorMode)
{
  ControlMappingConfig config;
  config.dead_zone = 0.1;
  config.saturation_zone = 0.6;
  config.normalization_mode = NormalizationMode::Vector;

  const Eigen::Vector3d command = mapLandmarkToCommand(
    Eigen::Vector3d(0.3, 0.4, 0.0),
    config);

  EXPECT_DOUBLE_EQ(command.x(), 0.48);
  EXPECT_DOUBLE_EQ(command.y(), 0.64);
  EXPECT_DOUBLE_EQ(command.z(), 0.0);
}

TEST(ControlMapping, DistinguishesAxisAndVectorDeadZones)
{
  const Eigen::Vector3d landmark(0.04, 0.04, 0.04);
  ControlMappingConfig config;
  config.dead_zone = 0.05;
  config.normalization_mode = NormalizationMode::Axis;
  EXPECT_TRUE(isInDeadZone(landmark, config));

  config.normalization_mode = NormalizationMode::Vector;
  EXPECT_FALSE(isInDeadZone(landmark, config));
}

TEST(ControlMapping, RejectsNonFiniteLandmarks)
{
  const Eigen::Vector3d landmark(
    std::numeric_limits<double>::quiet_NaN(),
    0.0,
    0.0);

  EXPECT_FALSE(isFiniteLandmark(landmark));
  EXPECT_EQ(
    mapLandmarkToCommand(landmark, ControlMappingConfig{}),
    Eigen::Vector3d::Zero());
}

}  // namespace
}  // namespace hand_joystick_interface
