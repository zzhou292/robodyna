// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ScreenValues.h"

namespace tl::fea::cin_timestep {
// The sole owner supplies startup-validated membership/role maps. On failure
// only invalid_node changes; output and all source arrays remain untouched.
TL_SURFACE_HD inline bool Screen(const Sources& source, double factor, Result& output,
    std::uint32_t& invalid_node) noexcept {
  if (!detail::CheckSources(source, factor)) return false;
  Result next;
  for (std::uint32_t node = 0; node < source.nodes; ++node) {
    invalid_node = node;
    if (!detail::EvaluateNode(source, factor, node, next)) return false;
  }
  if (!detail::EvaluateGroups(source, factor, next, invalid_node)) return false;
  next.valid = true;
  invalid_node = UINT32_MAX;
  output = next;
  return true;
}
} // namespace tl::fea::cin_timestep
