// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ForceChecks.h"

namespace tl::fea::solid18::law44 {
namespace detail {
struct HistoryWriter {
  TL_SOLID18_HD static Status Prepare(const Reference& reference, const Material& material,
      const HistoryValues& values, HistoryStamp stamp, History& output) noexcept {
    if (!ValidMaterial(reference,material) || !Nonnegative(stamp.time_s) ||
        !ValidHistory(reference,material,values)) return Status::InvalidInput;
    output.reference_ = reference;
    output.material_ = material;
    output.data_ = values;
    output.stamp_ = stamp;
    output.prepared_ = true;
    return Status::Success;
  }
};
}
TL_SOLID18_HD inline Status PreparePrescribedHistory(const Reference& reference,
    const Material& material, const HistoryValues& values, HistoryStamp stamp, History& output) noexcept {
  History next;
  const auto status = detail::HistoryWriter::Prepare(reference,material,values,stamp,next);
  if (status == Status::Success) output = next;
  return status;
}
namespace detail {
TL_SOLID18_HD inline void InitialHistoryValues(const Reference& reference,
    const Material& material, HistoryValues& values) noexcept {
  values = {};
  for (unsigned ip = 0; ip < 8; ++ip) {
    auto& p = values.point[ip];
    p.density_kg_m3 = material.material.density_kg_m3;
    p.storage_volume_m3 = reference.geometry().point[ip].initial_volume_m3;
    p.initial_volume_m3 = p.storage_volume_m3;
  }
  values.global.density_kg_m3 = reference.mass().initial_global_density_kg_m3;
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = reference.geometry().native_position_m[n];
    const auto& last = reference.geometry().native_position_m[7];
    values.saved_local_position_m[n] = {x.x-last.x,x.y-last.y,x.z-last.z};
  }
}
}  // namespace detail
TL_SOLID18_HD inline Status InitializeHistory(const Reference& reference,
    const Material& material, History& output) noexcept {
  if (!detail::ValidMaterial(reference,material)) return Status::InvalidInput;
  HistoryValues values;
  detail::InitialHistoryValues(reference,material,values);
  return PreparePrescribedHistory(reference,material,values,{},output);
}
}  // namespace tl::fea::solid18::law44
