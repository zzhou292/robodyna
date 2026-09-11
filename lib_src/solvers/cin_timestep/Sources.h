// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Rigid.h"

namespace tl::fea::cin_timestep {
struct Sources {
  const double* accepted = nullptr; // Existing x3/v3/w3/q4/reaction6 + group tail.
  const double* mass = nullptr;
  const double* inertia = nullptr;
  const double* translation = nullptr;
  const double* rotation = nullptr;
  const std::uint8_t* fixed_translation = nullptr;
  const std::uint8_t* fixed_rotation = nullptr;
  const std::uint8_t* rotation_present = nullptr; // Null means all present.
  const std::uint8_t* cin_secondary = nullptr;
  rigid::GroupDeviceView rigid;
  std::uint32_t nodes = 0;
  double previous_drift_dt = 0;
};
struct Result {
  double minimum_dt = std::numeric_limits<double>::max();
  std::uint32_t limiting_node = UINT32_MAX;
  std::uint32_t limiting_group = UINT32_MAX;
  bool valid = false;
};
TL_SURFACE_HD inline void Include(const ScalarLimit& limit, std::uint32_t node,
    std::uint32_t group, Result& result) noexcept {
  if (limit.bounded && limit.dt < result.minimum_dt) {
    result.minimum_dt = limit.dt;
    result.limiting_node = node;
    result.limiting_group = group;
  }
}
} // namespace tl::fea::cin_timestep
