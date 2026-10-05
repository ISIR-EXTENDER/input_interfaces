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

#ifndef HAND_JOYSTICK_INTERFACE__CONTROL_MAPPING_HPP_
#define HAND_JOYSTICK_INTERFACE__CONTROL_MAPPING_HPP_

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace hand_joystick_interface
{

constexpr std::size_t kHandLandmarkCount = 21;
constexpr std::size_t kTranslationAxisCount = 3;
constexpr std::size_t kDefaultLandmarkIndex = 8;

enum class NormalizationMode
{
  Axis,
  Vector
};

struct ControlMappingConfig
{
  double dead_zone{0.05};
  double saturation_zone{0.2};
  NormalizationMode normalization_mode{NormalizationMode::Vector};
  int output_round_decimals{2};
};

std::optional<NormalizationMode> parseNormalizationMode(const std::string & value);

bool isValidLandmarkIndex(std::int64_t index);

bool isFiniteLandmark(const Eigen::Vector3d & landmark);

bool isInDeadZone(
  const Eigen::Vector3d & landmark,
  const ControlMappingConfig & config);

Eigen::Vector3d mapLandmarkToCommand(
  const Eigen::Vector3d & landmark,
  const ControlMappingConfig & config);

}  // namespace hand_joystick_interface

#endif  // HAND_JOYSTICK_INTERFACE__CONTROL_MAPPING_HPP_
