// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Force.h"

namespace tl::fea::solid18::law44::detail {
// One unpublished workspace per concurrent caller. Inputs must be disjoint
// from it. A rejected call may leave partial scratch, never accepted state.
struct ForceScratch {
  ForceTrial trial;
  HistoryValues next;
  StartupGeometry geometry;
  History virgin;
  PrescribedInterval initial;
};
TL_SOLID18_HD inline Status EvaluateForceScratch(const Reference& reference,
    const History& accepted, const PrescribedInterval& interval,
    const Material& material, ForceScratch& scratch) noexcept {
  if (!ValidMaterial(reference,material) || !ValidInterval(accepted,interval) ||
      !SameReference(reference,accepted.reference()) ||
      !SameMaterial(material,accepted.material()) ||
      !ValidHistory(reference,material,accepted.data())) return Status::InvalidInput;
  return CalculateForceStaged(reference,accepted,interval,material,scratch.trial,
      scratch.next,scratch.geometry,false);
}
TL_SOLID18_HD inline Status InitializeForceScratch(const Reference& reference,
    const Material& material, Vec3 velocity, ForceScratch& scratch) noexcept {
  if (!solid18::detail::Finite(velocity) || !ValidMaterial(reference,material))
    return Status::InvalidInput;
  InitialHistoryValues(reference,material,scratch.next);
  const auto status = HistoryWriter::Prepare(reference,material,scratch.next,{},scratch.virgin);
  if (status != Status::Success) return status;
  auto& initial = scratch.initial;
  initial.base_time_s = 0;
  initial.dt_s = 0;
  initial.sample_index = 0;
  for (unsigned n = 0; n < 8; ++n) {
    initial.position_endpoint_m[n] = reference.input().position_m[n];
    initial.velocity_midpoint_m_s[n] = velocity;
  }
  return CalculateForceStaged(reference,scratch.virgin,initial,material,scratch.trial,
      scratch.next,scratch.geometry,true);
}
}  // namespace tl::fea::solid18::law44::detail
