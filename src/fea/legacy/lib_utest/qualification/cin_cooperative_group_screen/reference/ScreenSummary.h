// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_timestep/ScreenValues.h"
#include "../NodalStateLimits.h"
#include <type_traits>

namespace tl::fea::cin_advance::screen {
inline constexpr unsigned Threads = 128, MaximumBlocks = 256;
// Finite dt is selected, never added or atomically accumulated. An invalid
// domain row wins before any group suffix; ties use the original domain index.
struct Summary {
  double minimum_dt;
  std::uint32_t limiting_node;
  std::uint32_t invalid_node;
};
static_assert(std::is_trivial<Summary>::value, "Shared screen records need no constructors");
static_assert(sizeof(Summary) == 16, "Count complete typed screen scratch");
TL_SURFACE_HD inline Summary Empty() noexcept {
  return {std::numeric_limits<double>::max(), UINT32_MAX, UINT32_MAX};
}
TL_SURFACE_HD inline unsigned Blocks(std::size_t nodes) noexcept {
  if (!nodes || nodes > MaxActiveNodalStateNodes) return 0;
  const auto count = 1+(nodes-1)/Threads;
  return static_cast<unsigned>(count > MaximumBlocks ? MaximumBlocks : count);
}
TL_SURFACE_HD inline void Merge(Summary& a, const Summary& b) noexcept {
  if (b.invalid_node < a.invalid_node) a.invalid_node = b.invalid_node;
  // Empty's DBL_MAX/UINT32_MAX exactly represents the strict old sentinel.
  // An equal dt replaces only a later domain row, independent of tree order.
  if (b.minimum_dt < a.minimum_dt ||
      (b.minimum_dt == a.minimum_dt && b.limiting_node < a.limiting_node)) {
    a.minimum_dt = b.minimum_dt;
    a.limiting_node = b.limiting_node;
  }
}
TL_SURFACE_HD inline void Observe(const cin_timestep::Sources& source, double factor,
    std::uint32_t node, Summary& summary) noexcept {
  cin_timestep::Result result;
  if (!cin_timestep::detail::EvaluateNode(source, factor, node, result)) {
    if (node < summary.invalid_node) summary.invalid_node = node;
    return;
  }
  Merge(summary, {result.minimum_dt, result.limiting_node, UINT32_MAX});
}
// Called only after the original header checks. Restore the last visited
// ordinary index even when a malformed first group has not read a member yet.
TL_SURFACE_HD inline bool Complete(const cin_timestep::Sources& source, double factor,
    const Summary& summary, cin_timestep::Result& output, std::uint32_t& invalid_node) noexcept {
  if (summary.invalid_node != UINT32_MAX) {
    invalid_node = summary.invalid_node;
    return false;
  }
  cin_timestep::Result next;
  next.minimum_dt = summary.minimum_dt;
  next.limiting_node = summary.limiting_node;
  invalid_node = source.nodes-1;
  if (!cin_timestep::detail::EvaluateGroups(source, factor, next, invalid_node)) return false;
  next.valid = true;
  invalid_node = UINT32_MAX;
  output = next;
  return true;
}
} // namespace tl::fea::cin_advance::screen
