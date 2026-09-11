// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../../math/Fixed3.h"
#include <cstddef>
#include <cstdint>

namespace tl::fea::qeph::mapped {
inline constexpr unsigned ObserverThreads=128;
inline constexpr unsigned ObserverMaxBlocks=256;
inline constexpr unsigned ObserverChannels=8;
// Fixed source-index subsequences and a fixed binary merge tree. These are
// observations only; no element history, force, stiffness or owner state lives here.
struct ObserverSummary {
  double sum[ObserverChannels];
  double maximum_term;
  double minimum_area,minimum_thickness,minimum_dt;
  double maximum_displacement,maximum_strain,maximum_curvature;
  std::uint32_t material_count;
  bool serial;
};
static_assert(sizeof(ObserverSummary)==128,"Account the complete typed observer record");
static_assert((ObserverThreads&(ObserverThreads-1))==0,"Fixed binary tree");
TL_SURFACE_HD inline unsigned ObserverBlocks(std::size_t parents,std::size_t nodes) noexcept {
  const auto count=parents>nodes?parents:nodes;
  if(!count || parents>UINT32_MAX/4 || nodes>=UINT32_MAX) return 0;
  const auto blocks=1+(count-1)/ObserverThreads;
  return static_cast<unsigned>(blocks>ObserverMaxBlocks?ObserverMaxBlocks:blocks);
}
} // namespace tl::fea::qeph::mapped
