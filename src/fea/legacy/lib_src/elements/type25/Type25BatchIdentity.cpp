// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type25BatchIdentity.h"

namespace tl::fea::type25::batch_detail {
BatchDiagnostics InitialDiagnostics(const BatchConfig& c,std::uint64_t source,double dt) {
  BatchDiagnostics d;d.source_instance_id=source;d.owner_id=c.owner.owner_id;
  d.configuration_id=c.configuration_id;d.qualification_id=c.qualification_id;
  d.epoch=c.owner.epoch;d.time=c.owner.time;d.velocity_time=c.owner.velocity_time;
  d.phase=BatchPhase::Accepted;d.valid=true;d.element_count=c.element_count;
  d.active_count=c.element_count;d.minimum_native_dt=dt;return d;
}
bool SameDiagnostics(const BatchDiagnostics& a,const BatchDiagnostics& b) noexcept {
  if(a.source_instance_id!=b.source_instance_id||a.owner_id!=b.owner_id||a.configuration_id!=b.configuration_id||
     a.qualification_id!=b.qualification_id||a.epoch!=b.epoch||a.base_epoch!=b.base_epoch||a.attempt!=b.attempt||
     a.time!=b.time||a.base_time!=b.base_time||a.velocity_time!=b.velocity_time||a.base_velocity_time!=b.base_velocity_time||
     a.kick_dt!=b.kick_dt||a.phase!=b.phase||a.valid!=b.valid||a.has_completed_interval!=b.has_completed_interval||
     a.accepted_force_assembled!=b.accepted_force_assembled||a.element_count!=b.element_count||a.active_count!=b.active_count||
     a.newly_failed_count!=b.newly_failed_count||a.internal_kick_work!=b.internal_kick_work||
     a.internal_drift_work!=b.internal_drift_work||a.minimum_native_dt!=b.minimum_native_dt)return false;
  for(unsigned i=0;i<4;++i)if(a.internal_work_J[i]!=b.internal_work_J[i]||a.internal_work_increment_J[i]!=b.internal_work_increment_J[i])return false;
  return true;
}
} // namespace tl::fea::type25::batch_detail
