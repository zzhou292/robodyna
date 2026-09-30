// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"

namespace tlfea::contact::nodal_wall_mapped::response {
namespace d = nodal_wall_device_detail;
namespace fe = tl::fea;
using Code = NodalWallDeviceStatus;

TL_SURFACE_HD inline bool Ordinary(const d::Storage& storage, Sidecar side,
    unsigned row, double& maximum) {
  const auto& node = storage.result.nodes[row];
  const auto upper = ::fmax(node.stiffness.upper, node.stiffness.value);
  if (upper == 0) return true;
  const auto root = side.roots[row];
  if (root != UINT32_MAX) return root < side.groups;
  double rate = 0;
  if (!mass_detail::UpperProduct(upper, side.inverse[row], &rate)) return false;
  maximum = ::fmax(maximum, rate);
  return true;
}

TL_SURFACE_HD inline bool Rigid(const d::Storage& storage, Sidecar side,
    const fe::DeviceNodalKinematicsView& k, unsigned group) {
  side.traces[group] = 0;
  const auto begin = side.response.offsets[group];
  const auto end = side.response.offsets[group+1];
  if (begin > end || end > storage.model.node_count) return false;
  for (auto slot = begin; slot < end; ++slot) {
    const auto row = side.response.rows[slot];
    if (row >= storage.model.node_count) return false;
    const auto& node = storage.result.nodes[row];
    const auto upper = ::fmax(node.stiffness.upper, node.stiffness.value);
    if (upper == 0) continue;
    if (side.roots[row] != group) return false;
    const auto* x = k.position_xyz+3*node.node;
    RigidNormalResponse response;
    if (EvaluateRigidNormalResponse(side.bodies[group], {x[0], x[1], x[2]}, {-1, 0, 0}, response) != Status::kOk ||
        AccumulateRigidContactTrace(upper, response, side.traces[group]) != Status::kOk) return false;
  }
  return true;
}

TL_SURFACE_HD inline bool CompleteRate(d::Storage& storage, Sidecar side) {
  const auto failure = side.summary->parent_failure;
  // Later users of this nonoverlapping integer scratch see the original seed.
  side.summary->parent_failure = NoFailure;
  if (failure != NoFailure) return false;
  side.summary->rate = 0;
  for (std::size_t b = 0; b < side.response.block_count; ++b)
    side.summary->rate = ::fmax(side.summary->rate, side.response.maxima[b]);
  for (std::size_t g = 0; g < side.groups; ++g)
    side.summary->rate = ::fmax(side.summary->rate, side.traces[g]);
  storage.result.diagnostics.stiffness_rate_bound = side.summary->rate;
  return true;
}

TL_SURFACE_HD inline void CheckStep(d::Storage& storage, Sidecar side) {
  double step = 0, frequency = 0;
  if (!mass_detail::Upper(::sqrt(side.summary->rate), &frequency) ||
      !mass_detail::UpperProduct(storage.model.config.owner.fixed_dt, frequency, &step) || step >= 1.6)
    d::Fail(storage.control, Code::StepTooLarge);
}
} // namespace tlfea::contact::nodal_wall_mapped::response
