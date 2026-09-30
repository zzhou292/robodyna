// SPDX-License-Identifier: AGPL-3.0-or-later
#include "QbatBatchIdentity.h"

namespace tl::fea::qbat::batch_detail {
bool SameDiagnostics(const BatchDiagnostics& a,const BatchDiagnostics& b) noexcept {
  if(a.owner_id!=b.owner_id||a.configuration_id!=b.configuration_id||a.qualification_id!=b.qualification_id||
      a.epoch!=b.epoch||a.base_epoch!=b.base_epoch||a.attempt!=b.attempt||a.phase!=b.phase||a.usage!=b.usage||
      a.valid!=b.valid||a.has_completed_interval!=b.has_completed_interval||
      a.accepted_force_assembled!=b.accepted_force_assembled||a.element_count!=b.element_count||
      a.active_count!=b.active_count||a.newly_removed_count!=b.newly_removed_count) return false;
  const double x[]{a.time,a.base_time,a.velocity_time,a.base_velocity_time,a.kick_dt,
      a.internal_work_j[0],a.internal_work_j[1],a.internal_work_increment_j[0],a.internal_work_increment_j[1],
      a.plastic_work_j,a.plastic_work_increment_j,a.numerical_viscous_work_j,a.numerical_viscous_work_increment_j,
      a.minimum_area_ratio,a.minimum_thickness_ratio,a.maximum_displacement,a.maximum_absolute_strain,
      a.minimum_native_dt,a.internal_kick_work,a.internal_drift_work};
  const double y[]{b.time,b.base_time,b.velocity_time,b.base_velocity_time,b.kick_dt,
      b.internal_work_j[0],b.internal_work_j[1],b.internal_work_increment_j[0],b.internal_work_increment_j[1],
      b.plastic_work_j,b.plastic_work_increment_j,b.numerical_viscous_work_j,b.numerical_viscous_work_increment_j,
      b.minimum_area_ratio,b.minimum_thickness_ratio,b.maximum_displacement,b.maximum_absolute_strain,
      b.minimum_native_dt,b.internal_kick_work,b.internal_drift_work};
  for(unsigned i=0;i<sizeof(x)/sizeof(x[0]);++i) {
    if(!shell_startup_detail::SameBits(x[i],y[i])) return false;
  }
  return true;
}
BatchDiagnostics CandidateIdentity(const BatchConfig& config,const NodalPreparedView& view) noexcept {
  BatchDiagnostics result;
  result.owner_id=config.owner.owner_id;
  result.configuration_id=config.configuration_id;
  result.qualification_id=config.qualification_id;
  result.base_epoch=view.kinematics.base_epoch;
  result.epoch=result.base_epoch+1;
  result.attempt=view.attempt;
  result.base_time=view.base_time;
  result.time=view.proposed_time;
  result.base_velocity_time=view.base_velocity_time;
  result.velocity_time=view.velocity_time;
  result.kick_dt=view.kick_dt;
  result.phase=BatchPhase::Prepared;
  result.usage=config.usage;
  result.has_completed_interval=true;
  result.accepted_force_assembled=config.usage==BatchUsage::CoupledForces;
  return result;
}
} // namespace tl::fea::qbat::batch_detail
