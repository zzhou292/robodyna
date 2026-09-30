// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tl::fea::type45::resident_detail {
bool SameConfig(const BatchConfig& a,const BatchConfig& b) noexcept {
  return trial_identity::SameStamp(a.owner,b.owner) && a.configuration_id==b.configuration_id &&
      a.qualification_id==b.qualification_id && shell_startup_detail::SameStartup(a.startup,b.startup) &&
      a.profile==b.profile && a.cin_attachment_count==b.cin_attachment_count &&
      a.cin_witness_count==b.cin_witness_count && a.limits.max_joints==b.limits.max_joints &&
      a.limits.max_nodes==b.limits.max_nodes && a.limits.max_host_bytes==b.limits.max_host_bytes &&
      a.limits.max_device_bytes==b.limits.max_device_bytes;
}
bool SameDiagnostics(const BatchDiagnostics& a,const BatchDiagnostics& b) noexcept {
  using detail::Same;
  return a.source_instance_id==b.source_instance_id && a.owner_id==b.owner_id &&
      a.configuration_id==b.configuration_id && a.qualification_id==b.qualification_id &&
      a.epoch==b.epoch && a.base_epoch==b.base_epoch && a.attempt==b.attempt &&
      Same(a.time,b.time) && Same(a.base_time,b.base_time) && Same(a.velocity_time,b.velocity_time) &&
      Same(a.base_velocity_time,b.base_velocity_time) && Same(a.kick_dt,b.kick_dt) &&
      a.phase==b.phase && a.joint_count==b.joint_count &&
      Same(a.native_internal_work_increment_j,b.native_internal_work_increment_j) &&
      Same(a.internal_kick_work_j,b.internal_kick_work_j) && Same(a.internal_drift_work_j,b.internal_drift_work_j) &&
      a.valid==b.valid && a.has_completed_interval==b.has_completed_interval &&
      a.accepted_force_assembled==b.accepted_force_assembled &&
      a.automatic_stiffness_initialized==b.automatic_stiffness_initialized;
}
} // namespace tl::fea::type45::resident_detail
