// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18Force.h"

namespace tl::fea::solid18::detail {
// One unpublished workspace per concurrent caller. It is never an accepted
// history owner, and may contain partial values after rejection. Every input
// must be disjoint from this workspace. A successful call replaces its trial.
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
      !ValidHistory(reference,accepted.data())) return Status::InvalidInput;
  return CalculateForceStaged(reference,accepted,interval,material,scratch.trial,
      scratch.next,scratch.geometry,false);
}
TL_SOLID18_HD inline Status InitializeForceScratch(const Reference& reference,
    const Material& material, Vec3 velocity, ForceScratch& scratch) noexcept {
  if (!Finite(velocity) || !ValidMaterial(reference,material)) return Status::InvalidInput;
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
} // namespace tl::fea::solid18::detail
