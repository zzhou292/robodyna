#pragma once
#include "NodalWallContactDevice.h"
namespace tlfea::contact::nodal_wall_device_detail {
inline bool SameCertificate(Q4CertifiedIntegral a,Q4CertifiedIntegral b) {
  return a.value==b.value && a.lower==b.lower && a.upper==b.upper && a.error==b.error;
}
inline bool SameDiagnostics(const NodalWallDiagnostics& a,const NodalWallDiagnostics& b) {
  if (!a.valid || !b.valid || a.owner_id!=b.owner_id || a.configuration_id!=b.configuration_id ||
      a.qualification_id!=b.qualification_id || a.wall_binding_id!=b.wall_binding_id ||
      a.base_epoch!=b.base_epoch || a.attempt!=b.attempt || a.phase!=b.phase || a.scheme!=b.scheme ||
      a.velocity_phase!=b.velocity_phase || a.node_count!=b.node_count || a.parent_count!=b.parent_count ||
      !SameCertificate(a.resultant,b.resultant) || !SameCertificate(a.potential,b.potential)) return false;
  const double x[]{a.time,a.velocity_time,a.base_time,a.base_velocity_time,a.kick_dt,
      a.wall_reaction.x,a.wall_reaction.y,a.wall_reaction.z,a.wall_moment.x,a.wall_moment.y,a.wall_moment.z,
      a.surface_power,a.maximum_penetration,a.stiffness_rate_bound,a.base_potential,a.base_potential_error,
      a.potential_increment,a.kick_work,a.kick_work_roundoff,a.drift_work,a.drift_work_roundoff,
      a.conservative_defect,a.work_uncertainty,a.quadratic_work_upper,a.wall_kick_impulse,a.wall_kick_impulse_error,
      a.wall_kick_moment.x,a.wall_kick_moment.y,a.wall_kick_moment.z,
      a.wall_kick_moment_error.x,a.wall_kick_moment_error.y,a.wall_kick_moment_error.z};
  const double y[]{b.time,b.velocity_time,b.base_time,b.base_velocity_time,b.kick_dt,
      b.wall_reaction.x,b.wall_reaction.y,b.wall_reaction.z,b.wall_moment.x,b.wall_moment.y,b.wall_moment.z,
      b.surface_power,b.maximum_penetration,b.stiffness_rate_bound,b.base_potential,b.base_potential_error,
      b.potential_increment,b.kick_work,b.kick_work_roundoff,b.drift_work,b.drift_work_roundoff,
      b.conservative_defect,b.work_uncertainty,b.quadratic_work_upper,b.wall_kick_impulse,b.wall_kick_impulse_error,
      b.wall_kick_moment.x,b.wall_kick_moment.y,b.wall_kick_moment.z,
      b.wall_kick_moment_error.x,b.wall_kick_moment_error.y,b.wall_kick_moment_error.z};
  for (unsigned i=0;i<sizeof(x)/sizeof(*x);++i) if (x[i]!=y[i]) return false;
  return true;
}
} // namespace tlfea::contact::nodal_wall_device_detail
