// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::type13::batch_detail {
bool SameDiagnostics(const BatchDiagnostics& a, const BatchDiagnostics& b) noexcept {
  if (a.source_instance_id != b.source_instance_id || a.owner_id != b.owner_id ||
      a.configuration_id != b.configuration_id || a.qualification_id != b.qualification_id ||
      a.epoch != b.epoch || a.base_epoch != b.base_epoch || a.attempt != b.attempt ||
      a.time != b.time || a.base_time != b.base_time || a.velocity_time != b.velocity_time ||
      a.base_velocity_time != b.base_velocity_time || a.kick_dt != b.kick_dt ||
      a.phase != b.phase || a.valid != b.valid ||
      a.has_completed_interval != b.has_completed_interval ||
      a.accepted_force_assembled != b.accepted_force_assembled ||
      a.element_count != b.element_count || a.active_count != b.active_count ||
      a.newly_failed_count != b.newly_failed_count ||
      a.internal_kick_work_J != b.internal_kick_work_J ||
      a.internal_drift_work_J != b.internal_drift_work_J ||
      a.minimum_native_dt_s != b.minimum_native_dt_s) {
    return false;
  }
  for (unsigned k = 0; k < ChannelCount; ++k) {
    if (a.internal_work_J[k] != b.internal_work_J[k] ||
        a.internal_work_increment_J[k] != b.internal_work_increment_J[k]) {
      return false;
    }
  }
  return true;
}
} // namespace tl::fea::type13::batch_detail
