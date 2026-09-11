// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18ForceChecks.h"

namespace tl::fea::solid18 {
// Explicit finite prescribed history. This is not a native restart reader or
// a physical state owner. The prepared curve backing stays caller-owned and
// immutable, including across host/device rebasing by a future owner.
TL_SOLID18_HD inline Status PreparePrescribedHistory(const Reference& reference,
    const Material& material, const HistoryValues& values,
    HistoryStamp stamp, History& output) noexcept {
  if (!detail::ValidMaterial(reference,material) ||
      !tl::math::Finite(stamp.time_s) || stamp.time_s < 0 ||
      !detail::ValidHistory(reference,values)) return Status::InvalidInput;
  History next;
  next.reference_ = reference;
  next.material_ = material;
  next.data_ = values;
  next.stamp_ = stamp;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}

TL_SOLID18_HD inline Status InitializeHistory(const Reference& reference,
    const Material& material, History& output) noexcept {
  if (!detail::ValidMaterial(reference,material)) return Status::InvalidInput;
  HistoryValues values;
  for (unsigned ip = 0; ip < 8; ++ip) {
    auto& point = values.point[ip];
    point.density_kg_m3 = material.density_kg_m3;
    point.storage_volume_m3 = reference.geometry().point[ip].initial_volume_m3;
    point.initial_volume_m3 = point.storage_volume_m3;
  }
  values.global.density_kg_m3 = reference.mass().initial_global_density_kg_m3;
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = reference.geometry().native_position_m[n];
    const auto& last = reference.geometry().native_position_m[7];
    values.saved_local_position_m[n] = {x.x-last.x,x.y-last.y,x.z-last.z};
  }
  return PreparePrescribedHistory(reference,material,values,{},output);
}
}  // namespace tl::fea::solid18
