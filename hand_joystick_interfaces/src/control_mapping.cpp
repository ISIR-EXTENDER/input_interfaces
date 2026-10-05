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

#include "hand_joystick_interface/control_mapping.hpp"

#include <cmath>

#include "signal_processing/dead_zone.hpp"

namespace hand_joystick_interface
{

std::optional<NormalizationMode> parseNormalizationMode(const std::string & value)
{
  if (value == "axis") {
    return NormalizationMode::Axis;
  }
  if (value == "vector") {
    return NormalizationMode::Vector;
  }
  return std::nullopt;
}

bool isValidLandmarkIndex(std::int64_t index)
{
  return index >= 0 &&
         static_cast<std::size_t>(index) < kHandLandmarkCount;
}

bool isFiniteLandmark(const Eigen::Vector3d & landmark)
{
  return landmark.allFinite();
}

bool isInDeadZone(
  const Eigen::Vector3d & landmark,
  const ControlMappingConfig & config)
{
  if (!isFiniteLandmark(landmark)) {
    return false;
  }

  if (config.normalization_mode == NormalizationMode::Axis) {
    return (landmark.array().abs() <= std::abs(config.dead_zone)).all();
  }

  return signal_processing::isInsideNormDeadZone(
    landmark,
    config.dead_zone);
}

Eigen::Vector3d mapLandmarkToCommand(
  const Eigen::Vector3d & landmark,
  const ControlMappingConfig & config)
{
  if (!isFiniteLandmark(landmark) || isInDeadZone(landmark, config)) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d normalized;
  if (config.normalization_mode == NormalizationMode::Axis) {
    normalized = signal_processing::applyScaledDeadZonePerAxis(
      landmark,
      config.dead_zone,
      config.saturation_zone);
  } else {
    normalized = signal_processing::applyNormDeadZone(
      landmark,
      config.dead_zone,
      config.saturation_zone);
  }

  const double rounding_factor = std::pow(
    10.0,
    static_cast<double>(config.output_round_decimals));
  Eigen::Vector3d output;
  for (std::size_t index = 0; index < kTranslationAxisCount; ++index) {
    const auto input_index = static_cast<Eigen::Index>(index);
    const double rounded_value =
      std::round(normalized[input_index] * rounding_factor) /
      rounding_factor;
    output[static_cast<Eigen::Index>(index)] = rounded_value;
  }
  return output;
}

}  // namespace hand_joystick_interface
