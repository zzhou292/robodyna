// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../math/Fixed3.h"
#include <cstddef>
#include <cstdint>

namespace tl::fea::mapped_shell {
inline constexpr unsigned ObserverThreads = 128;
inline constexpr unsigned ObserverMaxBlocks = 256;
inline constexpr unsigned ObserverChannels = 8;
// Qualified QEPH layout; T3 leaves channels4/5 unused. These are observations,
// with no force, stiffness, material history or owner state in the record.
struct ObserverSummary {
  double sum[ObserverChannels];
  double maximum_term;
  double minimum_area, minimum_thickness, minimum_dt;
  double maximum_displacement, maximum_strain, maximum_curvature;
  std::uint32_t material_count;
  bool serial;
};
static_assert(sizeof(ObserverSummary) == 128, "Complete typed observer record");
static_assert((ObserverThreads & (ObserverThreads - 1)) == 0, "Fixed binary tree");
TL_SURFACE_HD inline unsigned ObserverBlocks(std::size_t parents, std::size_t nodes) noexcept {
  const auto count = parents > nodes ? parents : nodes;
  if (!count || parents > UINT32_MAX / 4 || nodes >= UINT32_MAX) return 0;
  const auto blocks = 1 + (count - 1) / ObserverThreads;
  return static_cast<unsigned>(blocks > ObserverMaxBlocks ? ObserverMaxBlocks : blocks);
}
} // namespace tl::fea::mapped_shell
