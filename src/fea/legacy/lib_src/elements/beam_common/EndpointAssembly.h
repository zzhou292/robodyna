// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellBatchFields.h"
#include "../../solvers/NodalForceAssembly.h"

namespace tl::fea::beam_endpoint {
// Complete disjoint owner destinations are already authenticated. Stage both
// scalar coefficients before invoking the existing atomic force/couple scatter.
// Success adds one positive native STI/STIR term to each distinct endpoint.
TL_SURFACE_HD inline NodalForceAssemblyStatus Accumulate(
    const std::size_t (&nodes)[2], const tl::math::Vec3 (&force)[2],
    const tl::math::Vec3 (&couple)[2], double translation, double rotation,
    DeviceNodalForceView view, double* sti, double* stir) noexcept {
  if (!sti || !stir || sti == stir || !tl::math::Finite(translation) || translation <= 0 ||
      !tl::math::Finite(rotation) || rotation <= 0)
    return NodalForceAssemblyStatus::InvalidView;
  if (nodes[0] == nodes[1] || nodes[0] >= view.node_count || nodes[1] >= view.node_count)
    return NodalForceAssemblyStatus::InvalidConnectivity;
  double next_t[2], next_r[2];
  for (unsigned n = 0; n < 2; ++n) {
    const auto node = nodes[n];
    next_t[n] = sti[node] + translation;
    next_r[n] = stir[node] + rotation;
    if (!tl::math::Finite(sti[node]) || sti[node] < 0 ||
        !tl::math::Finite(stir[node]) || stir[node] < 0 ||
        !tl::math::Finite(next_t[n]) || !tl::math::Finite(next_r[n]))
      return NodalForceAssemblyStatus::NonfiniteResult;
  }
  const auto status = AccumulateNodalForces<2>(nodes, force, couple, view);
  if (status != NodalForceAssemblyStatus::Success) return status;
  for (unsigned n = 0; n < 2; ++n) {
    sti[nodes[n]] = next_t[n];
    stir[nodes[n]] = next_r[n];
  }
  return NodalForceAssemblyStatus::Success;
}
} // namespace tl::fea::beam_endpoint
