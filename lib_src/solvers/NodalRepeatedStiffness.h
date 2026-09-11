// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalForceAssembly.h"

#if defined(__CUDACC__)
#define TL_SLOT_STIFFNESS_HD __host__ __device__
#else
#define TL_SLOT_STIFFNESS_HD
#endif
namespace tl::fea {
// Positive source-slot stiffness terms, in original slot order. The owner
// serializes writers and supplies disjoint input/destination storage. Source
// admission and the element's stiffness formula remain outside this operation.
// All consumed values and intermediate sums are checked before any write.
template<unsigned Count>
TL_SLOT_STIFFNESS_HD inline NodalForceAssemblyStatus AccumulateRepeatedNodalStiffness(
    const std::size_t* nodes, const double* increments, double* destination,
    std::size_t node_count) noexcept {
  static_assert(Count > 0 && Count <= 8, "Bounded source-slot stiffness scatter");
  if (!nodes || !increments || !destination || !node_count)
    return NodalForceAssemblyStatus::InvalidView;
  double staged[Count];
  for (unsigned n = 0; n < Count; ++n) {
    if (nodes[n] >= node_count) return NodalForceAssemblyStatus::InvalidConnectivity;
    double prior = destination[nodes[n]];
    // Most recent occurrence contains exactly the earlier source-slot sum.
    for (unsigned earlier = n; earlier > 0; --earlier) {
      if (nodes[earlier-1] == nodes[n]) { prior = staged[earlier-1]; break; }
    }
    if (!tl::math::Finite(prior) || prior < 0 ||
        !tl::math::Finite(increments[n]) || increments[n] <= 0)
      return NodalForceAssemblyStatus::NonfiniteResult;
    staged[n] = prior + increments[n];
    if (!tl::math::Finite(staged[n])) return NodalForceAssemblyStatus::NonfiniteResult;
  }
  for (unsigned n = 0; n < Count; ++n) destination[nodes[n]] = staged[n];
  return NodalForceAssemblyStatus::Success;
}
} // namespace tl::fea
#undef TL_SLOT_STIFFNESS_HD
