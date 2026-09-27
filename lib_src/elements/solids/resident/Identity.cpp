// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::solids::batch_detail {
bool SameConfig(const BatchConfig& a, const BatchConfig& b) noexcept {
  if (!trial_identity::SameStamp(a.owner, b.owner) ||
      a.configuration_id != b.configuration_id || a.qualification_id != b.qualification_id ||
      !shell_startup_detail::SameStartup(a.startup, b.startup) || a.profile != b.profile ||
      a.cin_attachment_count != b.cin_attachment_count || a.cin_witness_count != b.cin_witness_count)
    return false;
  const auto& x = a.limits;
  const auto& y = b.limits;
  return x.max_parents == y.max_parents && x.max_materials == y.max_materials &&
      x.max_curve_points == y.max_curve_points && x.max_nodes == y.max_nodes &&
      x.max_device_bytes == y.max_device_bytes && x.max_host_bytes == y.max_host_bytes;
}
bool SameDiagnostics(const BatchDiagnostics& a, const BatchDiagnostics& b) noexcept {
  using shell_startup_detail::SameBits;
  if (a.source_instance_id != b.source_instance_id || a.owner_id != b.owner_id ||
      a.configuration_id != b.configuration_id || a.qualification_id != b.qualification_id ||
      a.epoch != b.epoch || a.base_epoch != b.base_epoch || a.attempt != b.attempt ||
      !SameBits(a.time, b.time) || !SameBits(a.base_time, b.base_time) ||
      !SameBits(a.velocity_time, b.velocity_time) ||
      !SameBits(a.base_velocity_time, b.base_velocity_time) || !SameBits(a.kick_dt, b.kick_dt) ||
      a.phase != b.phase || a.valid != b.valid ||
      a.has_completed_interval != b.has_completed_interval ||
      a.accepted_force_assembled != b.accepted_force_assembled ||
      !SameBits(a.plastic_work_increment_j, b.plastic_work_increment_j) ||
      !SameBits(a.internal_kick_work_j, b.internal_kick_work_j) ||
      !SameBits(a.internal_drift_work_j, b.internal_drift_work_j) ||
      !SameBits(a.minimum_native_dt_s, b.minimum_native_dt_s)) return false;
  for (unsigned f = 0; f < 5; ++f) {
    if (a.parent_count[f] != b.parent_count[f] ||
        !SameBits(a.native_internal_work_increment_j[f], b.native_internal_work_increment_j[f]) ||
        !SameBits(a.physical_hourglass_work_increment_j[f], b.physical_hourglass_work_increment_j[f]) ||
        !SameBits(a.distortion_work_increment_j[f], b.distortion_work_increment_j[f]))
      return false;
  }
  return true;
}
} // namespace tl::fea::solids::batch_detail
